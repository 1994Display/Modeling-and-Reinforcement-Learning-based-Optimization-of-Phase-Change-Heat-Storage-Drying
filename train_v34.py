"""
v34 训练 + 转换一体化脚本
流程: 训练 PPO(5维动作) → 导出 ONNX → 转换 TFLite → 生成 C 头文件
用法: python train_v34.py
"""
import os
import sys

def main(total_steps=500_000):
    # 1) 训练 PPO（5 维动作空间）
    print("=" * 60)
    print("v34: 训练 PPO（新增加热片，动作 5 维）")
    print("=" * 60)
    from train_ppo import train, export_onnx

    ppo, history = train(total_steps=total_steps,
                         save_interval=10_000, log_interval=10_000)

    # 2) 导出 ONNX
    print("\n导出 ONNX 模型...")
    export_onnx(ppo.policy, "policy.onnx", obs_dim=5)

    # 3) 转换 TFLite + C 头文件
    print("\n转换 TFLite + 生成 C 头文件...")
    import convert_model
    convert_model.pt_to_tflite("policy_best.pt", "policy.tflite", obs_dim=5, act_dim=5)
    convert_model.tflite_to_c_header("policy.tflite", "policy_model.h")

    print("\n全部完成！")
    print("输出文件:")
    print("  - policy_best.pt   : PyTorch 最优模型 (5维动作)")
    print("  - policy_final.pt  : PyTorch 最终模型")
    print("  - policy.onnx      : ONNX 中间格式")
    print("  - policy.tflite    : TFLite float32 模型")
    print("  - policy_model.h   : C 头文件（供 ESP32 部署）")

if __name__ == "__main__":
    steps = int(sys.argv[1]) if len(sys.argv) > 1 else 500_000
    main(total_steps=steps)
