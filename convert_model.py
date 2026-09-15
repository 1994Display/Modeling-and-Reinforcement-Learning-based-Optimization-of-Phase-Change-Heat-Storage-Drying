"""
PyTorch 模型 → TensorFlow Lite (float32) + C 头文件
直接提取权重重建，不依赖 onnx_tf
输出: policy.tflite, policy_model.h
"""
import numpy as np
import torch
import tensorflow as tf


def pt_to_tflite(pt_path="policy_best.pt", tflite_path="policy.tflite",
                 obs_dim=5, act_dim=5, hidden=128, pt_path_alt="policy_final.pt",
                 aug_dim=7):
    """从 PyTorch 权重直接构建 TF 模型并导出 TFLite（含特征增强 5→7）"""
    from train_ppo import PolicyNet

    # 1) 加载 PyTorch 模型（优先最优模型，否则用最终模型）
    print("[1/4] Loading PyTorch model...")
    try:
        state_dict = torch.load(pt_path, map_location="cpu", weights_only=True)
        print(f"  Loaded: {pt_path}")
    except Exception as e:
        print(f"  {pt_path} failed ({e}), trying {pt_path_alt}...")
        state_dict = torch.load(pt_path_alt, map_location="cpu", weights_only=True)
        print(f"  Loaded: {pt_path_alt}")

    pt_model = PolicyNet(obs_dim, act_dim, hidden)
    pt_model.load_state_dict(state_dict)
    pt_model.eval()

    # 2) 提取权重 (net: 9→128→128→128, LN, mean: 128→4)
    print("[2/4] Extracting weights...")
    w1 = state_dict["net.0.weight"].cpu().numpy()    # (128, 9)
    b1 = state_dict["net.0.bias"].cpu().numpy()      # (128,)
    w2 = state_dict["net.2.weight"].cpu().numpy()    # (128, 128)
    b2 = state_dict["net.2.bias"].cpu().numpy()      # (128,)
    w3 = state_dict["net.4.weight"].cpu().numpy()    # (128, 128)  v33: 第三层
    b3 = state_dict["net.4.bias"].cpu().numpy()      # (128,)
    ln_gamma = state_dict["mean_ln.weight"].cpu().numpy()  # (128,)  v33: LayerNorm
    ln_beta  = state_dict["mean_ln.bias"].cpu().numpy()    # (128,)
    # LayerNorm eps: PyTorch 默认 1e-5
    ln_eps = 1e-5
    w_out = state_dict["mean.weight"].cpu().numpy()  # (5, 128)
    b_out = state_dict["mean.bias"].cpu().numpy()    # (5,)

    # 3) 构建等价的 TF 模型（含特征增强 + 3层ReLU + LayerNorm + sigmoid）
    print("[3/4] Building TF model (v34.9: 5-obs + 3-layer + LayerNorm)...")

    class PolicyTF(tf.Module):
        def __init__(self):
            super().__init__()
            self.w1 = tf.Variable(w1.T, dtype=tf.float32)         # (7, 128)
            self.b1 = tf.Variable(b1, dtype=tf.float32)
            self.w2 = tf.Variable(w2.T, dtype=tf.float32)         # (128, 128)
            self.b2 = tf.Variable(b2, dtype=tf.float32)
            self.w3 = tf.Variable(w3.T, dtype=tf.float32)         # (128, 128)  v33
            self.b3 = tf.Variable(b3, dtype=tf.float32)
            self.ln_gamma = tf.Variable(ln_gamma, dtype=tf.float32)  # (128,) v33
            self.ln_beta  = tf.Variable(ln_beta, dtype=tf.float32)   # (128,)
            self.w_out = tf.Variable(w_out.T, dtype=tf.float32)      # (128, 5)
            self.b_out = tf.Variable(b_out, dtype=tf.float32)

        def _layer_norm(self, x):
            """手动实现 LayerNorm (TF 无直接等价, 沿最后一维归一化)
            注意：用 diff*diff 代替 tf.square()，避免生成 SQUARED_DIFFERENCE 算子
            该算子在旧版 TFLite Micro (ESP32) 中可能不支持"""
            mean = tf.reduce_mean(x, axis=-1, keepdims=True)
            diff = x - mean
            var = tf.reduce_mean(diff * diff, axis=-1, keepdims=True)
            x_norm = (x - mean) / tf.sqrt(var + 1e-5)
            return x_norm * self.ln_gamma + self.ln_beta

        @tf.function(input_signature=[tf.TensorSpec(shape=[1, obs_dim], dtype=tf.float32)])
        def __call__(self, x):
            # 特征增强：5 维 → 7 维
            t = x[:, 0:1]
            t_step = tf.nn.sigmoid((t - 0.35) * 200.0)
            x_aug = tf.concat([x, t * t, t_step], axis=-1)
            # 3 层 MLP + ReLU
            h = tf.nn.relu(x_aug @ self.w1 + self.b1)
            h = tf.nn.relu(h @ self.w2 + self.b2)
            h = tf.nn.relu(h @ self.w3 + self.b3)          # v33: 第三层
            h_norm = self._layer_norm(h)                    # v33: LayerNorm
            y = 0.05 + 0.90 * tf.nn.sigmoid(h_norm @ self.w_out + self.b_out)
            return y

    tf_model = PolicyTF()

    # 运行一次初始化变量
    _ = tf_model(tf.constant(np.zeros((1, obs_dim), dtype=np.float32)))

    # 转换为 TFLite
    converter = tf.lite.TFLiteConverter.from_concrete_functions(
        [tf_model.__call__.get_concrete_function()], tf_model
    )
    tflite_model = converter.convert()

    # 4) 保存
    print("[4/4] Saving TFLite model...")
    with open(tflite_path, "wb") as f:
        f.write(tflite_model)

    # 模型信息
    interpreter = tf.lite.Interpreter(model_content=tflite_model)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    model_size_kb = len(tflite_model) / 1024

    print(f"\nTFLite model: {tflite_path}")
    print(f"  Size: {model_size_kb:.1f} KB")
    print(f"  Input: {input_details[0]['shape']} ({input_details[0]['dtype']})")
    print(f"  Output: {output_details[0]['shape']} ({output_details[0]['dtype']})")

    # 验证：对比 PyTorch 与 TFLite 输出（归一化输入, 5维观测）
    # [PCM温度, 干燥仓温度, 干燥仓湿度, 水箱温度, 环境温度常量]
    raw_test = np.array([[50.0, 45.0, 30.0, 25.0, 25.0]], dtype=np.float32)
    scale = np.array([[100.0, 100.0, 80.0, 100.0, 40.0]], dtype=np.float32)
    test_input = np.clip(raw_test / scale, 0, 1)
    with torch.no_grad():
        mean, _ = pt_model(torch.FloatTensor(test_input))
        pt_output = mean.numpy()[0]

    interpreter.set_tensor(input_details[0]['index'], test_input)
    interpreter.invoke()
    tflite_output = interpreter.get_tensor(output_details[0]['index'])[0]

    diff = np.abs(pt_output - tflite_output).max()
    print(f"  PyTorch output:  {pt_output}")
    print(f"  TFLite output:   {tflite_output}")
    print(f"  Max diff:        {diff:.6f} {'[OK]' if diff < 1e-4 else '[MISMATCH!]'}")

    return tflite_path


def tflite_to_c_header(tflite_path="policy.tflite", header_path="policy_model.h"):
    """将 TFLite 模型转为 C 头文件（直接嵌入 ESP32 代码）"""
    with open(tflite_path, "rb") as f:
        model_data = f.read()

    hex_array = ", ".join(f"0x{b:02x}" for b in model_data)

    header = f"""// 干燥系统策略网络模型 (TFLite float32)
// 自动生成，请勿手动编辑
// 模型大小: {len(model_data) / 1024:.1f} KB

#ifndef POLICY_MODEL_H
#define POLICY_MODEL_H

#include <stdint.h>

const unsigned int policy_model_len = {len(model_data)};
const unsigned char policy_model_data[] = {{
    {hex_array}
}};

#endif  // POLICY_MODEL_H
"""

    with open(header_path, "w") as f:
        f.write(header)

    print(f"C header generated: {header_path}")


if __name__ == "__main__":
    pt_to_tflite("policy_best.pt", "policy.tflite")
    tflite_to_c_header("policy.tflite", "policy_model.h")
    print("\nDone!")
