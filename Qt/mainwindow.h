#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QTimer>
#include <QPlainTextEdit>
#include "joysitck_xbox360.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    Joysitck_Xbox360 *pJst;

    QTimer *timerRefreshUI;

    QSerialPort *serialPort1;

private:
    void SbusGenerator(Joysitck_Xbox360* jst);

    void LogWnd_WriteLine(QPlainTextEdit *logWnd, QString line);
    void LogWnd_WriteLine(QString line);
    QPlainTextEdit *logWnd;

private slots:
    void on_refreshUI_timeout();

private slots:

    void axisLeftXChanged(qint32 value);
    void axisLeftYChanged(qint32 value);
    void axisRightXChanged(qint32 value);
    void axisRightYChanged(qint32 value);

    void buttonAChanged(bool value);
    void buttonBChanged(bool value);
    void buttonXChanged(bool value);
    void buttonYChanged(bool value);

    void buttonUpChanged(bool value);
    void buttonDownChanged(bool value);
    void buttonLeftChanged(bool value);
    void buttonRightChanged(bool value);

    void buttonLeftBreak(bool value);
    void buttonRightBreak(bool value);

    void buttonLeftTrigger(bool value);
    void buttonRightTrigger(bool value);

    void connectedChanged(bool value);
//    void deviceIdChanged(int value);
//    void nameChanged(QString value);

    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
};
#endif // MAINWINDOW_H
