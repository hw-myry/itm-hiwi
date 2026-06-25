#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void on_btnConnect_clicked();
    void on_btnLedOn_clicked();
    void on_btnLedOff_clicked();

    void on_sendAngleButton_clicked();

    // 一个按钮：发送全部参数，并保存到 ESP32 Flash
    void on_btnSaveParameter_clicked();

private:
    Ui::MainWindow *ui;
    QTcpSocket *socket = nullptr;

    bool ledOn = false;

    // 用来缓存 TCP 接收数据，防止一次 readAll 收到多行或半行
    QString rxBuffer;

    void sendRGB();
    void addLog(const QString &msg);

    // 统一发送命令
    void sendCommand(const QString &cmd, bool showLog = true);

    // 解析 ESP32 返回数据
    void handleEsp32Line(const QString &line);
    void updateConfigEditsFromLine(const QString &line);
    QString valueFromLine(const QString &line, const QString &key) const;
};

#endif // MAINWINDOW_H