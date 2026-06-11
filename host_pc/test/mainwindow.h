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

private:
    Ui::MainWindow *ui;
    QTcpSocket *socket;

    bool ledOn = false;

    void sendRGB();
    void addLog(const QString &msg);
};

#endif // MAINWINDOW_H