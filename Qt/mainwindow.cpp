#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QMessageBox>
#include <QScrollBar>
#include <QSerialPortInfo>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    serialPort1 = new QSerialPort;

    //连接设备
    pJst = new Joysitck_Xbox360;
    connect(this->pJst,SIGNAL(finished()),this->pJst,SLOT(deleteLater()));
    pJst->start();


    timerRefreshUI = new QTimer();
    connect(this->timerRefreshUI,SIGNAL(timeout()),this,SLOT(on_refreshUI_timeout()));
    timerRefreshUI->start(20);

    //Log窗口只读
    ui->plainTextEdit_logWnd->setReadOnly(true);
    this->logWnd = ui->plainTextEdit_logWnd;

    LogWnd_WriteLine(QString("Joystick start with %1 axis.").arg(pJst->joynums));
    LogWnd_WriteLine("Sys Start.");

    foreach( const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        LogWnd_WriteLine(QString("serial port system loction: %1").arg(info.systemLocation()));
        LogWnd_WriteLine(QString("serial port name: %1").arg(info.portName()));
#ifdef _WIN32
        ui->lineEdit_serialPortName->setText(info.portName());
#else
        ui->lineEdit_serialPortName->setText(info.systemLocation());
#endif
    }
}

MainWindow::~MainWindow()
{
    if(serialPort1->isOpen())
    {
        serialPort1->close();
        qDebug()<<"Quit and Close Serial Port.";
    }
    delete ui;
}

void MainWindow::SbusGenerator(Joysitck_Xbox360 *jst)
{
    const int SERIAL_PORT_SND_LEN = 15;
    QByteArray qbaSerialPortData(SERIAL_PORT_SND_LEN,0x00);

    qbaSerialPortData[0]=quint8(0xdf);

    //左右摇杆
    qbaSerialPortData[1]=(jst->leftAxisY>>5)>>8;qbaSerialPortData[2]=(jst->leftAxisY>>5)&0xff;
    qbaSerialPortData[3]=(jst->leftAxisX>>5)>>8;qbaSerialPortData[4]=(jst->leftAxisX>>5)&0xff;
    qbaSerialPortData[5]=(jst->rightAxisY>>5)>>8;qbaSerialPortData[6]=(jst->rightAxisY>>5)&0xff;
    qbaSerialPortData[7]=(jst->rightAxisX>>5)>>8;qbaSerialPortData[8]=(jst->rightAxisX>>5)&0xff;

    //qbaSerialPortData[9]=(jst->buttons&0xff)>>1;qbaSerialPortData[10]=(jst->buttons&0xff)<<1;
    qbaSerialPortData[9]=0;qbaSerialPortData[10]=0;

    //Kill Button
    if(jst->buttonRightTrigger==true)
    {
        ui->radioButton_kill->setChecked(true);
        qbaSerialPortData[11]=quint8(0x3f);qbaSerialPortData[12]=quint8(0xff);
    }
    else
    {
        ui->radioButton_kill->setChecked(false);
        qbaSerialPortData[11]=quint8(0x00);qbaSerialPortData[12]=quint8(0x00);
    }

    //Flight Mode
    //0 292 585 877 1169 1461 1753
    quint16 flightModeTable[]={292,585,877,1169,1461,1753};
    if(ui->radioButton_flightMode1->isChecked()) {qbaSerialPortData[13]=(flightModeTable[0])>>8;qbaSerialPortData[14]=(flightModeTable[0])&0xff;}
    if(ui->radioButton_flightMode2->isChecked()) {qbaSerialPortData[13]=(flightModeTable[1])>>8;qbaSerialPortData[14]=(flightModeTable[1])&0xff;}
    if(ui->radioButton_flightMode3->isChecked()) {qbaSerialPortData[13]=(flightModeTable[2])>>8;qbaSerialPortData[14]=(flightModeTable[2])&0xff;}
    if(ui->radioButton_flightMode4->isChecked()) {qbaSerialPortData[13]=(flightModeTable[3])>>8;qbaSerialPortData[14]=(flightModeTable[3])&0xff;}
    if(ui->radioButton_flightMode5->isChecked()) {qbaSerialPortData[13]=(flightModeTable[4])>>8;qbaSerialPortData[14]=(flightModeTable[4])&0xff;}
    if(ui->radioButton_flightMode6->isChecked()) {qbaSerialPortData[13]=(flightModeTable[5])>>8;qbaSerialPortData[14]=(flightModeTable[5])&0xff;}

    if(serialPort1->isOpen())
    {
        serialPort1->write(qbaSerialPortData,qbaSerialPortData.size());
        serialPort1->flush();
    }
}

void MainWindow::LogWnd_WriteLine(QPlainTextEdit *logWnd, QString line)
{
    //移动光标到最后一行
    logWnd->moveCursor(QTextCursor::End,QTextCursor::MoveAnchor);
    //文本数量过大就清空
    if(logWnd->toPlainText().size()>1025*8)
    {
        logWnd->clear();
    }
    //插入新的一行文本
    logWnd->insertPlainText(line+'\n');
    //滚动条滚动到底部
    QScrollBar *scrollbar = logWnd->verticalScrollBar();
    if(scrollbar)
    {
        scrollbar->setSliderPosition(scrollbar->maximum());
    }
}

void MainWindow::LogWnd_WriteLine(QString line)
{
    //移动光标到最后一行
    this->logWnd->moveCursor(QTextCursor::End,QTextCursor::MoveAnchor);
    //文本数量过大就清空
    if(this->logWnd->toPlainText().size()>1025*8)
    {
        this->logWnd->clear();
    }
    //插入新的一行文本
    this->logWnd->insertPlainText(line+'\n');
    //滚动条滚动到底部
    QScrollBar *scrollbar = this->logWnd->verticalScrollBar();
    if(scrollbar)
    {
        scrollbar->setSliderPosition(scrollbar->maximum());
    }
}

void MainWindow::on_refreshUI_timeout()
{
    axisRightXChanged(this->pJst->rightAxisX);
    axisRightYChanged(this->pJst->rightAxisY);
    axisLeftXChanged(this->pJst->leftAxisX);
    axisLeftYChanged(this->pJst->leftAxisY);

    QString buttons = QString("%1").arg(this->pJst->buttons);
    ui->lineEdit_buttons->setText(buttons);

    QString pov = QString("%1").arg(this->pJst->POV);
    ui->lineEdit_POV->setText(pov);
    buttonUpChanged(this->pJst->buttonUp);
    buttonDownChanged(this->pJst->buttonDown);
    buttonRightChanged(this->pJst->buttonRight);
    buttonLeftChanged(this->pJst->buttonLeft);


    buttonLeftBreak(this->pJst->buttonLeftBreak);
    buttonRightBreak(this->pJst->buttonRightBreak);

    buttonLeftTrigger(this->pJst->buttonLeftTrigger);
    buttonRightTrigger(this->pJst->buttonRightTrigger);

    buttonAChanged(this->pJst->buttonA);
    buttonBChanged(this->pJst->buttonB);
    buttonXChanged(this->pJst->buttonX);
    buttonYChanged(this->pJst->buttonY);

    this->SbusGenerator(pJst);
}

//右边摇杆X轴，往右为正
void MainWindow::axisRightXChanged(qint32 value)
{
    ui->label_axisRightX->setText(QString::number(value));
    ui->progressBar_rightAxisX->setValue(value);

}
//右边摇杆Y轴，往下为正
void MainWindow::axisRightYChanged(qint32 value)
{
    ui->label_axisRightY->setText(QString::number(value));
    ui->progressBar_rightAxisY->setValue(value);
}
//左边摇杆X轴，往右为正
void MainWindow::axisLeftXChanged(qint32 value)
{
    ui->label_axisLeftX->setText(QString::number(value));
    ui->progressBar_leftAxisX->setValue(value);

}
//左边摇杆Y轴，往下为正
void MainWindow::axisLeftYChanged(qint32 value)
{
    ui->label_axisLeftY->setText(QString::number(value));
    ui->progressBar_leftAxisY->setValue(value);
}

//A键
void MainWindow::buttonAChanged(bool value)
{
    if(value==true) ui->label_btnA->setStyleSheet("background-color:blue");
    else ui->label_btnA->setStyleSheet("");
}
//B键
void MainWindow::buttonBChanged(bool value)
{
    if(value==true) ui->label_btnB->setStyleSheet("background-color:red");
    else ui->label_btnB->setStyleSheet("");
}
//X键
void MainWindow::buttonXChanged(bool value)
{
    if(value==true) ui->label_btnX->setStyleSheet("background-color:pink");
    else ui->label_btnX->setStyleSheet("");
}
//Y键
void MainWindow::buttonYChanged(bool value)
{
    if(value==true) ui->label_btnY->setStyleSheet("background-color:green");
    else ui->label_btnY->setStyleSheet("");
}

//左边上
void MainWindow::buttonUpChanged(bool value)
{
    if(value==true) ui->label_btnUp->setStyleSheet("background-color:green");
    else ui->label_btnUp->setStyleSheet("");
}
//左边下
void MainWindow::buttonDownChanged(bool value)
{
    if(value==true) ui->label_btnDown->setStyleSheet("background-color:green");
    else ui->label_btnDown->setStyleSheet("");
}
//左边左
void MainWindow::buttonLeftChanged(bool value)
{
    if(value==true) ui->label_btnLeft->setStyleSheet("background-color:green");
    else ui->label_btnLeft->setStyleSheet("");
}
//左边右
void MainWindow::buttonRightChanged(bool value)
{
    if(value==true) ui->label_btnRight->setStyleSheet("background-color:green");
    else ui->label_btnRight->setStyleSheet("");
}

//LeftBreak键
void MainWindow::buttonLeftBreak(bool value)
{
    if(value==true) ui->label_btnLeftBreak->setStyleSheet("background-color:green");
    else ui->label_btnLeftBreak->setStyleSheet("");
}
//RightBreak键
void MainWindow::buttonRightBreak(bool value)
{
    if(value==true) ui->label_btnRightBreak->setStyleSheet("background-color:green");
    else ui->label_btnRightBreak->setStyleSheet("");
}

void MainWindow::buttonRightTrigger(bool value)
{
    if(value==true) ui->label_btnRightTrigger->setStyleSheet("background-color:green");
    else ui->label_btnRightTrigger->setStyleSheet("");
}

void MainWindow::buttonLeftTrigger(bool value)
{
    if(value==true) ui->label_btnLeftTrigger->setStyleSheet("background-color:green");
    else ui->label_btnLeftTrigger->setStyleSheet("");
}

void MainWindow::connectedChanged(bool value)
{
    qDebug()<<"connectedChanged"<<value;
    LogWnd_WriteLine("Joystick Connection Changed.");
}

void MainWindow::on_pushButton_clicked()
{
    pJst->okToQuit();
}

void MainWindow::on_pushButton_2_clicked()
{
    serialPort1->setPortName(ui->lineEdit_serialPortName->text());
    if(!serialPort1->open(QIODevice::ReadWrite))
    {
        //QMessageBox::warning(this,"Failed","Failed to Serial Port.",QMessageBox::Ok);
        LogWnd_WriteLine("Failed to Serial Port.");
        return;
    }
    serialPort1->setBaudRate(115200);
    serialPort1->setParity(QSerialPort::NoParity);
    serialPort1->setStopBits(QSerialPort::OneStop);
    serialPort1->setFlowControl(QSerialPort::NoFlowControl);
    serialPort1->setDataBits(QSerialPort::Data8);
    qDebug()<<"serial port start OK.";
    LogWnd_WriteLine("serial port start OK.");
}
