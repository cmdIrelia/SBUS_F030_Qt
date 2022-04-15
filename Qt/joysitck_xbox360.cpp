#include "joysitck_xbox360.h"

#include <QDebug>

Joysitck_Xbox360::Joysitck_Xbox360()
{
    joyinfoex.dwSize = sizeof (JOYINFOEX);
    joyinfoex.dwFlags = JOY_RETURNALL;
    joynums = joyGetNumDevs();
    qDebug()<<"Current we have: "<<QString::number(joynums)<<" Joystick(s).";
}

void Joysitck_Xbox360::run()
{
    qDebug()<<"Running...";
    while (1) {
        msleep(10);
        if(bShouldQuit)
        {
            qDebug()<<"Quit Thread";
            break;
        }

        if(joynums>=1)
        {
            joyreturn = joyGetPosEx(JOYSTICKID1, &joyinfoex);
            if(joyreturn == JOYERR_NOERROR)
            {
                this->rightAxisX = joyinfoex.dwZpos;
                this->rightAxisY = 65535 - joyinfoex.dwRpos;
                this->leftAxisX = joyinfoex.dwXpos;
                this->leftAxisY = 65535 - joyinfoex.dwYpos;

                if(joyinfoex.dwButtons&64) buttonLeftBreak=true;
                else buttonLeftBreak=false;

                if(joyinfoex.dwButtons&128) buttonRightBreak=true;
                else buttonRightBreak=false;


                this->buttons = joyinfoex.dwButtons;

                this->buttonA = joyinfoex.dwButtons&0x04;
                this->buttonB = joyinfoex.dwButtons&0x02;
                this->buttonX = joyinfoex.dwButtons&0x08;
                this->buttonY = joyinfoex.dwButtons&0x01;

                this->POV = joyinfoex.dwPOV;
                this->buttonUp = (joyinfoex.dwPOV == 0);
                this->buttonDown = (joyinfoex.dwPOV == 18000);
                this->buttonLeft = (joyinfoex.dwPOV == 27000);
                this->buttonRight = (joyinfoex.dwPOV == 9000);

                this->buttonRightTrigger = (joyinfoex.dwButtons & 32);
                this->buttonLeftTrigger = (joyinfoex.dwButtons & 16);

            }
            else
            {
                qDebug()<<"Joystick failed due to: "<<joyreturn;
            }
        }
    }
}

void Joysitck_Xbox360::okToQuit()
{
    this->bShouldQuit = true;
}
