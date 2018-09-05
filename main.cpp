#include "mbed.h"
/*Network libraries*/
#include "EthernetInterface.h"
#include "MQTTNetwork.h"
#include "MQTTClient.h"
#include "MQTTmbed.h"
/*ModbusRTU libraries*/
#include "BufferedSerial.h"
#include "ModbusMaster.h"

#define CLOUDMQTT

//LED interface
DigitalOut led_1(PD_12);
DigitalOut led_2(PD_13);
DigitalOut led_3(PD_14);
DigitalOut led_4(PD_15);

// Network interface
EthernetInterface net;
MQTTNetwork mqtt_network(&net);
MQTT::Client<MQTTNetwork,Countdown,100,10> mqtt(mqtt_network);
int rc;

#ifdef MY_PC
    const char* mqtt_broker = "192.168.0.13";
    int mqtt_port = 1883;
#endif    
#ifdef CLOUDMQTT
    const char* mqtt_broker = "m14.cloudmqtt.com";
    int mqtt_port = 16409;
#endif

/*Modbus variables*/
BufferedSerial modbus_serial(PD_5,PA_3);
ModbusMaster slave_1,slave_2;
Serial pc(PA_9,PA_10,115200);

/*Threads declaration*/
Thread Thread_1000ms(osPriorityNormal,OS_STACK_SIZE,NULL,"1000ms");
Thread Thread_Slave1(osPriorityNormal,OS_STACK_SIZE,NULL,"Slave1");
Thread Thread_Slave2(osPriorityNormal,OS_STACK_SIZE,NULL,"Slave2");

/*Mutex declaration*/
Mutex ModbusMutex;
Mutex MQTTMutex;

/*Functions declaration*/
void ReadSlave1_Routine();
void ReadSlave2_Routine();
void Task_1000ms();
void Net_Init();
void MQTT_Init();
void Modbus_Init();

#ifdef USE_TFT
void TFT_Init();
#endif

void mqtt_publish_num(uint16_t mb_data, const char* mqtt_topic)
{
    MQTTMutex.lock();
    char buf[100];
    MQTT::Message message;
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    sprintf(buf,"%d",mb_data);
    message.payload = (void*)buf;
    message.payloadlen = strlen(buf);
    mqtt.publish(mqtt_topic, message);
    MQTTMutex.unlock();
}

void mqtt_publish_str(const char* topic,const char* log_msg)
{
    MQTTMutex.lock();
    MQTT::Message message;
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*)log_msg;
    message.payloadlen = strlen(log_msg);
    mqtt.publish(topic, message);
    MQTTMutex.unlock();
}

void MessageHandle(MQTT::MessageData& md)
{
    MQTT::Message &message = md.message;
    char topic[md.topicName.lenstring.len + 1];
    char payload[message.payloadlen + 1];
    sprintf(topic, "%.*s", md.topicName.lenstring.len, md.topicName.lenstring.data);
    sprintf(payload,"%.*s",message.payloadlen,(char*)message.payload);
    //pc.printf("Message arrived: qos %d, retained %d, dup %d, packetid %d\r\n", message.qos, message.retained, message.dup, message.id);
    pc.printf("%s: %s\r\r\n",topic,payload);

    uint8_t result;
    uint16_t data = atoi(payload);

    if(strcmp(topic,"write/control") == 0)
    {   
        
        ModbusMutex.lock(); 
        result = slave_1.writeSingleRegister(0x2000,data); //0x2000 is control address
        ModbusMutex.unlock(); 

        if (result == slave_1.ku8MBSuccess)
        {
            switch(data)
            {
                case 1:
                    mqtt_publish_str("logging_slave1","Forward Success");
                    break;
                case 2:
                    mqtt_publish_str("logging_slave1","Reverse Success");
                    break;
                case 5:
                    mqtt_publish_str("logging_slave1","Stop Success");
                    break;
                case 6:
                    mqtt_publish_str("logging_slave1","Coast Stop Success");
                    break;
            }
        }
        else
        {
            switch(data)
            {
                case 1:
                    mqtt_publish_str("logging_slave1","Forward Error");
                    break;
                case 2:
                    mqtt_publish_str("logging_slave1","Reverse Error");
                    break;
                case 5:
                    mqtt_publish_str("logging_slave1","Stop Error");
                    break;
                case 6:
                    mqtt_publish_str("logging_slave1","Coast Stop Error");
                    break;
            }
        }
    }

    if(strcmp(topic,"write/frequency") == 0)
    {
        ModbusMutex.lock();
        result = slave_1.writeSingleRegister(0x2001,data); //0x2001 is frequency address
        ModbusMutex.unlock();

        if (result == slave_1.ku8MBSuccess)
        {
            mqtt_publish_str("logging_slave1","Frequency set Success");
        }
        else
        {
            mqtt_publish_str("logging_slave1","Frequency set Error");
        }
    }

    // if(strcmp(topic,"write/PIDsetting") == 0)
    // {
    //     ModbusMutex.lock();
    //     result = slave_1.writeSingleRegister(0x2002,data); 
    //     ModbusMutex.unlock();

    //     if (result == slave_1.ku8MBSuccess)
    //     {
    //         mqtt_publish_str("logging_slave1","PID Success");
    //     }
    //     else
    //     {
    //         mqtt_publish_str("logging_slave1","PID Error");
    //     }
    // }

    if(strcmp(topic,"write/led2") == 0) led_2 = data;
    if(strcmp(topic,"write/led3") == 0) led_3 = data;
    if(strcmp(topic,"write/led4") == 0) led_4 = data;
}

int main()
{   
    wait(1);
    pc.printf("=========================\r\r\n");
    pc.printf("===Mbed 5.9 ModbusMQTT===\r\r\n");
    pc.printf("=========================\r\r\n");
  
    Net_Init();
    MQTT_Init();
    Modbus_Init();
    
    /*Start threads*/
    Thread_Slave1.start(ReadSlave1_Routine);
    Thread_Slave2.start(ReadSlave2_Routine);
    Thread_1000ms.start(Task_1000ms);

    while(1)
    {
        if(mqtt.yield(1000L) != 0)
        {
            while(1)
            {
                led_1 = 1;
                Thread::wait(100);
                led_1 = 0;
                Thread::wait(100);
            }
        }
    }
}

void ReadSlave1_Routine()
{
    /*Read from Slave and publish*/
    uint8_t rd_result;
    uint16_t MB_data;
    while(true)
    {
        ModbusMutex.lock();
        rd_result = slave_1.readHoldingRegisters(0x3001, 9);
        ModbusMutex.unlock();

        if (rd_result == slave_1.ku8MBSuccess)
        {
            MB_data = slave_1.getResponseBuffer(0);
            mqtt_publish_num(MB_data,"read/frequency");
            MB_data = slave_1.getResponseBuffer(1);
            mqtt_publish_num(MB_data,"read/bus_volt");
            MB_data = slave_1.getResponseBuffer(2);
            mqtt_publish_num(MB_data,"read/out_volt");
            MB_data = slave_1.getResponseBuffer(3);
            mqtt_publish_num(MB_data,"out_current");
            MB_data = slave_1.getResponseBuffer(4);
            mqtt_publish_num(MB_data,"read/operation_speed");
            MB_data = slave_1.getResponseBuffer(5);
            mqtt_publish_num(MB_data,"read/out_power");
            MB_data = slave_1.getResponseBuffer(6);
            mqtt_publish_num(MB_data,"read/out_torque");
            MB_data = slave_1.getResponseBuffer(7);
            mqtt_publish_num(MB_data,"read/PID_setting");
            MB_data = slave_1.getResponseBuffer(8);
            mqtt_publish_num(MB_data,"read/PID_feedback");

            mqtt_publish_str("logging_slave1","Read Success");
        }
        else
        {
            mqtt_publish_str("logging_slave1","Read Failed");
        }  
        Thread::wait(200);  
    } 
}

void ReadSlave2_Routine()
{
    uint8_t rd_result;
    uint16_t MB_data;
    while(true)
    {
        ModbusMutex.lock();
        rd_result = slave_2.readHoldingRegisters(0x0000, 2);
        ModbusMutex.unlock();
        if (rd_result == slave_2.ku8MBSuccess)
        {
            MB_data = slave_2.getResponseBuffer(0);
            mqtt_publish_num(MB_data,"read/temperature");
            MB_data = slave_2.getResponseBuffer(1);
            mqtt_publish_num(MB_data,"read/humidity");

            mqtt_publish_str("logging_slave2","Read Success");
        }
        else
        {
            mqtt_publish_str("logging_slave2","Read Failed");
        }  
        Thread::wait(2000);
    }
}

void Task_1000ms()
{
    static uint32_t counter = 0;
    while(1)
    {
        char counter_buffer[10];
        sprintf(counter_buffer,"%ld",counter++);
        mqtt_publish_str("logging_counter",counter_buffer);
        Thread::wait(1000);
    }
    
}

void Net_Init()
{
    net.connect();
    const char *ip = net.get_ip_address();
    const char *netmask = net.get_netmask();
    const char *gateway = net.get_gateway();
    pc.printf("IP address: %s\r\r\n", ip ? ip : "None");
    pc.printf("Netmask: %s\r\r\n", netmask ? netmask : "None");
    pc.printf("Gateway: %s\r\r\n", gateway ? gateway : "None");
    pc.printf("Connecting to %s:%d\r\r\n", mqtt_broker, mqtt_port);
    rc = mqtt_network.connect(mqtt_broker, mqtt_port);
    if(rc != 0) pc.printf("Failed to get network [Error %d]\r\r\n", rc);
    else pc.printf("Connected to network!\r\r\n");
}

void MQTT_Init()
{
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    MQTTPacket_willOptions LWT = MQTTPacket_willOptions_initializer;

    LWT.topicName.cstring = "status";
    LWT.message.cstring = "Disconnected";

    data.MQTTVersion = 3;
    data.clientID.cstring = "MQBus";
    data.will = LWT;
    
    #ifdef CLOUDMQTT
        data.username.cstring = "jrxanwut";
        data.password.cstring = "uDU6C6XHHsFF";
    #endif

    rc = mqtt.connect(data);
    if(rc != 0) pc.printf("Failed to connect to MQTT [Error %d]\r\r\n", rc);
    else pc.printf("Connected to MQTT Broker!\r\r\n");

    rc =mqtt.subscribe("logging", MQTT::QOS0, MessageHandle);
    rc =mqtt.subscribe("write/control", MQTT::QOS0, MessageHandle);
    rc =mqtt.subscribe("write/frequency", MQTT::QOS0, MessageHandle);
    rc =mqtt.subscribe("write/led2", MQTT::QOS0, MessageHandle);
    rc =mqtt.subscribe("write/led3", MQTT::QOS0, MessageHandle);
    rc =mqtt.subscribe("write/led4", MQTT::QOS0, MessageHandle);

    if(rc != 0) pc.printf("Failed to subcribed [Error %d]\r\r\n", rc);
    else{
        pc.printf("Subcribed successfully!\r\r\n");
    }
    mqtt_publish_str("status","Connected");
}
void Modbus_Init()
{
    modbus_serial.baud(57600);
    modbus_serial.format(8,Serial::None,2);
    slave_1.begin(1,modbus_serial);
    slave_2.begin(2,modbus_serial);
}   
