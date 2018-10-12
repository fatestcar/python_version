//
// Created by 刘瑷玮 on 2018/10/12.
//
#include "GPIOlib.h"
extern "C"
{
int init(){
    return GPIO::init();
}

int controlLeft(int direction,int speed){
    return GPIO::controlLeft(direction,speed);
}
int controlRight(int direction,int speed){
    return GPIO::controlRight(direction, speed);
}
int stopLeft(){
    return GPIO::stopLeft();
}
int stopRight(){
    return GPIO::stopRight();
}

int resetCounter(){
    return GPIO::resetCounter();
}
void getCounter(int *countLeft,int *countRight){

}

int turnTo(int angle){
    return GPIO::turnTo(angle);
}

void delay(int milliseconds){
    GPIO::delay(milliseconds);
}
}