#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

/*
 * 引脚定义
 *
 * IIC屏
 * SCL:15
 * SDA:2
 *
 * MAX30105
 * SCL:22
 * SDA:21
 *
 * 震动马达
 * IN:12
 *
 * */




uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid


//iic驱动方式
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 2, /* reset=*/ U8X8_PIN_NONE);
//这里是替换成其他引脚
MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

void setup(void) {
    pinMode(13,OUTPUT);
    digitalWrite(13, LOW);

    Serial.begin(115200);//打开串口，便于代码调试
    u8g2.begin();//初始化屏幕
    u8g2.setFont(u8g2_font_ncenB08_tr);//设置屏幕默认字体
    // 初始化传感器
    while (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //使用默认的I2C端口，400kHz速度
    {
        u8g2.drawStr(0, 15, "Checking");//在屏幕上显示"Checking"
        u8g2.sendBuffer();//将缓存区的字符串传入屏幕
    }
    u8g2.clearBuffer();//清除屏幕
    Serial.println("Place your index finger on the sensor with steady pressure.");//调试信息

    particleSensor.setup(); //使用默认设置配置传感器
    particleSensor.setPulseAmplitudeRed(0x0A); //将红色LED转低以指示传感器正在运行
    particleSensor.setPulseAmplitudeGreen(0); //关闭绿色LED


}

typedef struct _last{
    int last_beat;
}last;

last last_value;//创建所需变量的结构体

bool send_wait=1;

int i = 0 ;

//采用一个函数主要是因为esp32的静态区较小，过多的全局变量可能会报错
void loop(void) {
    long irValue = particleSensor.getIR();//获取传感器的IR值

    if (checkForBeat(irValue) == true)//检查IR值是否正常
    {
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
            rates[rateSpot++] = (byte)beatsPerMinute; //将这个读数存储在数组中
            rateSpot %= RATE_SIZE;

            //取读数的平均值
            beatAvg = 0;
            for (byte x = 0 ; x < RATE_SIZE ; x++)
                beatAvg += rates[x];
            beatAvg /= RATE_SIZE;
        }
    }
    Serial.print("\nIR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);


    bufferLength = 50; //缓冲长度为100，存储以25sps运行的4秒样本

    //读取前50个样本，并确定信号范围
    if ( i < bufferLength )
    {

        while (particleSensor.available() == false) //检查是否有新数据
            particleSensor.check(); //检查传感器是否有新的数据

        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = irValue;
        particleSensor.nextSample(); //我们已经完成了这个样品，请转到下一个样品（进入下一个循环）

        //send samples and calculation result to terminal program through UART
        Serial.print(F("red="));
        Serial.print(redBuffer[i], DEC);
        Serial.print(F(", ir="));
        Serial.print(irBuffer[i], DEC);

        Serial.print(F(", HR="));
        Serial.print(heartRate, DEC);

        Serial.print(F(", HRvalid="));
        Serial.print(validHeartRate, DEC);

        Serial.print(F(", SPO2="));
        Serial.print(spo2, DEC);

        Serial.print(F(", SPO2Valid="));
        Serial.println(validSPO2, DEC);//串口打印调试信息
        i++;
    }else{
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);//计算血氧饱和度的核心算法
        i=0;
    }


    if (irValue < 80000){
        u8g2.clear();
        float temperature = particleSensor.readTemperature();
        u8g2.drawStr(0, 15, " No finger?");
        u8g2.drawStr(50, 50, "temperature");
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(50, 60, String(temperature-10).c_str());//在无手指时空闲时间显示温度（这里减去的是传感器自产热）
        u8g2.sendBuffer();
        Serial.print("\nNo finger?");//提示用户手指不在传感器上
        send_wait=1;
    }else{
        if(beatAvg>60&&beatAvg!=last_value.last_beat&&validSPO2){
            u8g2.clear();
            last_value.last_beat=beatAvg;
            u8g2.setFont(u8g2_font_ncenB08_tr);
            u8g2.drawStr(0, 15, "Oxygen Saturation:");
            u8g2.setFont(u8g2_font_ncenB10_tr);
            u8g2.drawStr(0, 30, String(spo2).c_str());
            u8g2.drawStr(30, 30, "%");//显示血氧值

            u8g2.drawStr(0, 45, "Heart Rate:");
            u8g2.drawStr(0, 60, String(beatAvg).c_str());
            u8g2.drawStr(30, 60, "BPM");//显示心率
            u8g2.sendBuffer();

            digitalWrite(13, HIGH);//这个是震动马达的拉高
            delay(2000);//延长震动时间
            digitalWrite(13, LOW);//停止震动
        }else{
            if(send_wait){
                u8g2.clear();
                u8g2.drawStr(0, 15, "wating");//提示用户需要等待
                u8g2.sendBuffer();
                send_wait=0;
            }
        }
    }


}

