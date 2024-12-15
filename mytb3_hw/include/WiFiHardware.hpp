#ifndef __WIFI_HARDWARE_H__
#define __WIFI_HARDWARE_H__

#include <WiFi.h>
#include <Arduino.h>

class WiFiHardware
{
    public:
    WiFiHardware(){};


    void init()
    {
        this->client.connect(this->server, this->port);
    }   

    void setConnection(IPAddress &server, int port)
    {
        this->server = server;
        this->port   = port;
    }

    int read()
    {
        if (this->client.connected()) return client.read();

        this->client.connect(this->server, this->port);
        return -1;
    }

    void write(uint8_t *data, int length)
    {
        for (int i = 0; i < length; i++) client.write(data[i]);
    }

    unsigned long time()
    {
        return millis();
    }

    protected:
    WiFiClient client;
    IPAddress server;
    uint16_t port;
};

#endif /* __WIFI_HARDWARE_H__ */