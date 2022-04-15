#ifndef JOYSITCK_XBOX360_H
#define JOYSITCK_XBOX360_H

#include <QThread>

#include <Windows.h>
#include <WinUser.h>
#include <mmsystem.h>

class Joysitck_Xbox360 : public QThread
{
public:
    Joysitck_Xbox360();

    qint32 leftAxisX{0},leftAxisY{0};
    qint32 rightAxisX{0},rightAxisY{0};

    bool buttonA,buttonB,buttonX,buttonY;
    bool buttonUp,buttonDown,buttonLeft,buttonRight;

    quint32 buttons,POV;

    bool buttonLeftBreak,buttonRightBreak;
    bool buttonLeftTrigger,buttonRightTrigger;

    UINT joynums;

protected:
    JOYINFO joyinfo;
    JOYINFOEX joyinfoex;
    MMRESULT joyreturn;


protected:
    void run() override;

public slots:
    void okToQuit();

private:
    bool bShouldQuit{false};
};

#endif // JOYSITCK_XBOX360_H
