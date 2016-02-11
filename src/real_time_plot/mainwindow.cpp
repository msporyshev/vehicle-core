#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
#include <cmath>
#include <algorithm>

#include <QMessageBox>
#include <QFile>

#include <ros/ros.h>

#define _USE_MATH_DEFINES

using namespace std;

const int MainWindow::ipc_timer_delay = 100;
const QString MainWindow::eth_connect = "169.254.100.2";
const QString MainWindow::config_file = "../../build/bin/down/config/motion.yml";
const string NODE_NAME = "real_time_plot";

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    target(0.),
    stab_value(0.),
    is_sended(false)
{
    ui->setupUi(this);

    ui->stabilize_box->addItem(QString("Depth"));
    ui->stabilize_box->addItem(QString("Heading"));
    ui->stabilize_box->addItem(QString("Pitch"));
    ui->stabilize_box->addItem(QString("X"));
    ui->stabilize_box->addItem(QString("Y"));

    ui->movemode_box->addItem(QString("HOVER"));
    ui->movemode_box->addItem(QString("CRUISE"));
    ui->movemode_box->addItem(QString("AUTO"));

    init_graph();
    update_coefs(0);

    modes = {
        {"HOVER", MoveMode::HOVER},
        {"CRUISE", MoveMode::CRUISE},
        {"AUTO", MoveMode::AUTO}
    };

    ipc_timer = new QTimer(this);
    QObject::connect(ipc_timer, SIGNAL(timeout()), this, SLOT(slot_ipc_timer()));
    QObject::connect(ui->button_send, SIGNAL(clicked()), this, SLOT(send_clicked()));
    QObject::connect(ui->button_stop, SIGNAL(clicked()), this, SLOT(stop_clicked()));
    QObject::connect(ui->stabilize_box, SIGNAL(currentIndexChanged(int)), this, SLOT(update_coefs(int)));
    QObject::connect(ipc_timer, SIGNAL(timeout()), this, SLOT(draw_graph()));
    ipc_timer->start(ipc_timer_delay);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete motion_client;
    delete com_;
}

void MainWindow::init_ipc()
{
    com_->subscribe("navig", &MainWindow::handle_angles, this);
    com_->subscribe("navig", &MainWindow::handle_pos, this);
    com_->subscribe("navig", &MainWindow::handle_depth, this);
}

void MainWindow::handle_angles(const navig::MsgNavigAngles& msg)
{
    angles_ = msg;
    auto text = ui->stabilize_box->currentText();
    if (text == "Heading") {
        stab_value = angles_.heading;
    } else if (text == "Pitch") {
        stab_value = angles_.pitch;
    }

    if (is_sended) {
        draw_graph();
    }
}

void MainWindow::handle_depth(const navig::MsgNavigDepth& msg)
{
    depth_ = msg;
    auto text = ui->stabilize_box->currentText();
    if (text == "Depth") {
        stab_value = depth_.depth;
    }

    if (is_sended) {
        draw_graph();
    }
}

void MainWindow::handle_pos(const navig::MsgNavigPosition& msg)
{
    pos_ = msg;
    auto text = ui->stabilize_box->currentText();
    if(text == "X") {
        stab_value = pos_.x;
    } else if(text == "Y") {
        stab_value = pos_.y;
    }

    if (is_sended) {
        draw_graph();
    }
}

void MainWindow::init_graph()
{
    ui->plot->addGraph();
    ui->plot->graph(0)->setPen(QPen(Qt::blue));
    ui->plot->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));
    ui->plot->graph(0)->setAntialiasedFill(false);
    ui->plot->addGraph();
    ui->plot->graph(1)->setPen(QPen(Qt::red));
    ui->plot->graph(0)->setChannelFillGraph(ui->plot->graph(1));

    ui->plot->addGraph();
    ui->plot->graph(2)->setPen(QPen(Qt::blue));
    ui->plot->graph(2)->setLineStyle(QCPGraph::lsNone);
    ui->plot->graph(2)->setScatterStyle(QCPScatterStyle::ssDisc);

    ui->plot->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    ui->plot->xAxis->setDateTimeFormat("hh:mm:ss");
    ui->plot->xAxis->setAutoTickStep(false);
    ui->plot->xAxis->setTickStep(1);
    ui->plot->axisRect()->setupFullAxesBox();

    ui->plot->replot();
}

void MainWindow::slot_ipc_timer()
{
    ros::spinOnce();
}

void MainWindow::send_clicked()
{
    if(ui->line_value->text().isEmpty()) {
        show_message(ui->label_value->text() + " is empty!");
        return;
    }
    if(ui->line_timeout->text().isEmpty()) {
        show_message(ui->label_timeout->text() + " is empty!");
    }

    double p(ui->spin_p->value()), i(ui->spin_i->value()), d(ui->spin_d->value());
    double value(ui->line_value->text().toDouble()), timeout(ui->line_timeout->text().toDouble());

    auto text = ui->stabilize_box->currentText();

    if(text == "Depth") {
        motion_client->fix_depth(value, timeout, p, i, d, WaitMode::DONT_WAIT);
    } else if(text == "Heading") {
        motion_client->fix_heading(value, timeout, p, i, d, WaitMode::DONT_WAIT);
    } else if(text == "Pitch") {
        motion_client->fix_pitch(value, timeout, p, i, d, WaitMode::DONT_WAIT);
    } else if(text == "X") {
        motion_client->fix_position(MakePoint2(value, 0.), modes[ui->movemode_box->currentText().toStdString()],
                timeout, p, i, d, 0, 0, 0, WaitMode::DONT_WAIT);
    } else if(text == "Y") {
        motion_client->fix_position(MakePoint2(0., value), modes[ui->movemode_box->currentText().toStdString()],
                timeout, 0, 0, 0, p, i, d, WaitMode::DONT_WAIT);
    }

    is_sended = true;
    target = value;
}

void MainWindow::stop_clicked()
{
    motion_client->unfix_all();
}

void MainWindow::set_coefs(QTextStream& in, QString text) {
    while(!in.atEnd()) {
        QString line = in.readLine();
        if(line.toStdString().find(text.toStdString()) != string::npos) {
            ui->spin_p->setValue(line.split(":").at(1).toDouble());
            in >> line >> line;
            ui->spin_i->setValue(line.toDouble());
            in >> line >> line;
            ui->spin_d->setValue(line.toDouble());
            break;
        }
    }
}

void MainWindow::update_coefs(int cur_pos)
{
    auto cur_text = ui->stabilize_box->currentText();

    QFile file(config_file);

    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return;
    }

    QTextStream in(&file);

    if(cur_text == "X" || cur_text == "Y") {
        while(!in.atEnd()) {
            QString line = in.readLine();
            if(line.toStdString().find("position:") != string::npos) {
                if(cur_text == "X") {
                    set_coefs(in, "fwd");
                }
                if(cur_text == "Y") {
                    set_coefs(in, "side");
                }
                break;
            }

        }
    } else {
        while(!in.atEnd()) {
            QString line = in.readLine();
            if(line.toStdString().find(cur_text.toLower().toStdString() + ":") != string::npos) {
                QString k;
                in >> k >> k;
                ui->spin_p->setValue(k.toDouble());
                in >> k >> k;
                cout << k.toStdString() << endl;
                ui->spin_i->setValue(k.toDouble());
                in >> k >> k;
                cout << k.toStdString() << endl;
                ui->spin_d->setValue(k.toDouble());
                break;
            }
        }
    }
    file.close();
}

void MainWindow::connect_to_vehicle_ipc(int argc, char* argv[])
{
    com_ = new ipc::Communicator(ipc::init(argc, argv, NODE_NAME));
    // cout << "Connecting to vehicle ipc on " << eth_connect.toStdString() << ". Result: " << res << endl;

    cout << "Successfully connected to " << argv[1] << endl;
    init_ipc();
    motion_client = new RobosubConfMotionClient(*com_);
}

void MainWindow::draw_graph()
{
    double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
    static double lastPointKey = 0;
    if (key-lastPointKey > 0.01)
    {
      ui->plot->graph(0)->addData(key, stab_value);
      ui->plot->graph(1)->addData(key, target);

      ui->plot->graph(2)->clearData();
      ui->plot->graph(2)->addData(key, stab_value);

      ui->plot->graph(0)->removeDataBefore(key-8);
      ui->plot->graph(1)->removeDataBefore(key-8);

      ui->plot->graph(0)->rescaleValueAxis();
      ui->plot->graph(1)->rescaleValueAxis(true);
      lastPointKey = key;
    }
    ui->plot->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
    ui->plot->replot();
}

void MainWindow::show_message(QString str)
{
    QMessageBox mb;
    mb.setText(QString::fromUtf8(str.toLatin1().constData()));
    mb.exec();
}
