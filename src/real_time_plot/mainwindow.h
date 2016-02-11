#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QString>
#include <qcustomplot.h>

#include <string>
#include <map>

#include <libipc/ipc.h>
#include <navig/MsgNavigAngles.h>
#include <navig/MsgNavigDepth.h>
#include <navig/MsgNavigPosition.h>
#include "motion/motion_client/robosub_conf_motion_client.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

    QTimer* ipc_timer;
    navig::MsgNavigAngles angles_;
    navig::MsgNavigDepth depth_;
    navig::MsgNavigPosition pos_;
    RobosubConfMotionClient* motion_client;
    ipc::Communicator* com_;
    double target, stab_value;
    bool is_sended;

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void init_graph();
    void connect_to_vehicle_ipc(int argc, char* argv[]);

    static const int ipc_timer_delay;
    static const QString eth_connect;
    static const QString config_file;

private slots:
    void slot_ipc_timer();
    void send_clicked();
    void stop_clicked();
    void draw_graph();
    void update_coefs(int cur_pos);

private:
    Ui::MainWindow *ui;

    void handle_angles(const navig::MsgNavigAngles& msg);
    void handle_depth(const navig::MsgNavigDepth& msg);
    void handle_pos(const navig::MsgNavigPosition& msg);

    void init_ipc();
    void show_message(QString str);
    void set_coefs(QTextStream &in, QString text);

    std::map<std::string, MoveMode> modes;
};

#endif // MAINWINDOW_H
