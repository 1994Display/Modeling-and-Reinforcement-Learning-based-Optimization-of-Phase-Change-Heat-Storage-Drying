程序的设计上，我们要运用OneWrie库，此库不属于Arduino的基本库，帖子后面附件会附带OneWrie库和DallasTemperature库，将压缩文件解压到libraries文件里即可。\
单独使用OneWrie库函数来试验。

==接口定义==

![144217pokh8w9l31sz899y](media/image1.jpeg){width="5.208333333333333in"
height="4.53125in"}

（各型号DS1820的引脚定义）

 

==Arduino 接线方法==

![TB2pMpGXai5V1BjSszcXXaDLXXa\_!!14857792](media/image2.jpeg){width="5.208333333333333in"
height="2.78125in"}

接线方式：红线（VCC，3.3V-5.5V）和黑线（GND）引脚分别接Arduino
5V和GND，黄线（DATA信号线）接Arduino PIN 10.  
其中黑线和黄线之间接的电阻为4.7K。

![DS18B20forArduino](media/image3.jpeg){width="5.729166666666667in"
height="2.6979166666666665in"}

接线方式：VD和GND引脚分别接Arduino 5V和GND，DQ接Arduino PIN 10.  
其中接的电阻为10K。

**(温湿度传感器模块上已经内置有上拉电阻了，无需再接电阻）**

![18B20tantou](media/image4.jpeg){width="5.729166666666667in"
height="3.1458333333333335in"}

接线方式：红线（VCC，3.3V-5.5V）和黑线（GND）引脚分别接Arduino
5V和GND，黄线（DATA信号线）接Arduino PIN 10.  
其中红线和黄线之间接的电阻为4.7K。

**(温湿度传感器探头需要接一个4.7KΩ的上拉电阻）**

**探头参数：**

优质不锈钢管封装  防水 防潮 防生锈

不锈钢头(6\*50mm)，引线100cm

每个探头经过严格测试

3.0V～5.5V供电

9～12位可调分辨率

测量温度范围为-55 ° C至+125 ℃ 。华氏相当于是67 ° F到257华氏度 -10 °
C至+85 ° C范围内精度为±0.5 ° C

使用范围：

1
.该产品适用于冷冻库，粮仓，储罐，电讯机房，电力机房，电缆线槽等测温和控制领域

2\. 轴瓦，缸体，纺机，空调，等狭小空间工业设备测温和控制。

3\. 汽车空调、冰箱、冷柜、以及中低温干燥箱等。

4 .供热/制冷管道热量计量，中央空调分户热能计量和工业领域测温和控制

输出引线：红色(VCC)，黄色(DATA)，黑色(GND)
