#include <string>
#include <iostream>
#include <sstream>
#include <cassert>

#include <QString>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QPixmap>

#ifndef QCOM
#include "networking.hpp"
#endif

#include "settings.hpp"
#include "widgets/input.hpp"
#include "widgets/toggle.hpp"
#include "widgets/offroad_alerts.hpp"

#include "common/params.h"
#include "common/util.h"

#include "selfdrive/hardware/hw.h"

QFrame* horizontal_line(QWidget* parent = 0){
  QFrame* line = new QFrame(parent);
  line->setFrameShape(QFrame::StyledPanel);
  line->setStyleSheet("margin-left: 40px; margin-right: 40px; border-width: 1px; border-bottom-style: solid; border-color: gray;");
  line->setFixedHeight(2);
  return line;
}

QWidget* labelWidget(QString labelName, QString labelContent){
  QHBoxLayout* labelLayout = new QHBoxLayout;
  labelLayout->addWidget(new QLabel(labelName), 0, Qt::AlignLeft);
  QLabel* paramContent = new QLabel(labelContent);
  paramContent->setStyleSheet("color: #aaaaaa");
  labelLayout->addWidget(paramContent, 0, Qt::AlignRight);
  QWidget* labelWidget = new QWidget;
  labelWidget->setLayout(labelLayout);
  return labelWidget;
}

ParamsToggle::ParamsToggle(QString param, QString title, QString description, QString icon_path, QWidget *parent): QFrame(parent) , param(param) {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setSpacing(50);

  // Parameter image
  if (icon_path.length()) {
    QPixmap pix(icon_path);
    QLabel *icon = new QLabel();
    icon->setPixmap(pix.scaledToWidth(80, Qt::SmoothTransformation));
    icon->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
    layout->addWidget(icon);
  } else {
    layout->addSpacing(80);
  }

  // Name of the parameter
  QLabel *label = new QLabel(title);
  label->setStyleSheet(R"(font-size: 50px;)");
  layout->addWidget(label);

  // toggle switch
  Toggle *toggle = new Toggle(this);
  toggle->setFixedSize(150, 100);
  layout->addWidget(toggle);
  QObject::connect(toggle, SIGNAL(stateChanged(int)), this, SLOT(checkboxClicked(int)));

  // set initial state from param
  if (Params().read_db_bool(param.toStdString().c_str())) {
    toggle->togglePosition();
  }

  setLayout(layout);
}

void ParamsToggle::checkboxClicked(int state) {
  char value = state ? '1': '0';
  Params().write_db_value(param.toStdString().c_str(), &value, 1);
}

QWidget * toggles_panel() {
  QVBoxLayout *toggles_list = new QVBoxLayout();
  toggles_list->setMargin(50);

  toggles_list->addWidget(new ParamsToggle("OpenpilotEnabledToggle",
                                            "오픈파일럿 사용",
                                            "어댑티브 크루즈 컨트롤 및 차선 유지 지원을 위해 오픈파일럿 시스템을 사용하십시오. 이 기능을 사용하려면 항상 주의를 기울여야 합니다. 이 설정을 변경하는 것은 자동차의 전원이 꺼졌을 때 적용됩니다.",
                                            "../assets/offroad/icon_openpilot.png"
                                              ));
  toggles_list->addWidget(horizontal_line());
  toggles_list->addWidget(new ParamsToggle("LaneChangeEnabled",
                                            "차선변경 보조 사용",
                                            "주변의 안전을 확인한 후 방향 지시등을 활성화하고 스티어링 휠을 원하는 차선 쪽으로 부드럽게 밀어 오픈파일럿으로 차선 변경 보조를 수행하십시오. 오픈파일럿은 차선 변경이 안전한지 확인할 수 없습니다. 이 기능을 사용하려면 주변 환경을 지속적으로 관찰해야 합니다.",
                                            "../assets/offroad/icon_road.png"
                                              ));
  toggles_list->addWidget(horizontal_line());
  toggles_list->addWidget(new ParamsToggle("IsLdwEnabled",
                                            "차선이탈 경보 사용",
                                            "50km/h이상의 속도로 주행하는 동안 방향 지시등이 활성화되지 않은 상태에서 차량이 감지된 차선 위를 넘어갈 경우 원래 차선으로 다시 방향을 전환하도록 경고를 보냅니다.",
                                            "../assets/offroad/icon_warning.png"
                                              ));
  toggles_list->addWidget(horizontal_line());
  toggles_list->addWidget(new ParamsToggle("RecordFront",
                                            "운전자 영상 녹화 및 업로드",
                                            "운전자 모니터링 카메라에서 데이터를 업로드하고 운전자 모니터링 알고리즘을 개선하십시오.",
                                            "../assets/offroad/icon_network.png"
                                            ));
  toggles_list->addWidget(horizontal_line());
  toggles_list->addWidget(new ParamsToggle("IsRHD",
                                            "우핸들 운전방식 사용",
                                            "오픈파일럿이 좌측 교통 규칙을 준수하도록 허용하고 우측 운전석에서 운전자 모니터링을 수행하십시오.",
                                            "../assets/offroad/icon_openpilot_mirrored.png"
                                            ));
  toggles_list->addWidget(horizontal_line());
  toggles_list->addWidget(new ParamsToggle("IsMetric",
                                            "미터법 사용",
                                            "mi/h 대신 km/h 단위로 속도를 표시합니다.",
                                            "../assets/offroad/icon_metric.png"
                                            ));
  toggles_list->addWidget(horizontal_line());
  toggles_list->addWidget(new ParamsToggle("CommunityFeaturesToggle",
                                            "커뮤니티 기능 사용",
                                            "comma.ai에서 유지 또는 지원하지 않고 표준 안전 모델에 부합하는 것으로 확인되지 않은 오픈 소스 커뮤니티의 기능을 사용하십시오. 이러한 기능에는 커뮤니티 지원 자동차와 커뮤니티 지원 하드웨어가 포함됩니다. 이러한 기능을 사용할 때는 각별히 주의해야 합니다.",
                                            "../assets/offroad/icon_shell.png"
                                            ));

  QWidget *widget = new QWidget;
  widget->setLayout(toggles_list);
  return widget;
}

QWidget * community_panel() {
  QVBoxLayout *toggles_list = new QVBoxLayout();
  toggles_list->setMargin(50);

  toggles_list->addWidget(new ParamsToggle("커뮤니티 기능",
                                            "커뮤니티 기능 사용",
                                            "커뮤니티의 오픈 소스를 사용합니다. 콤마의 정규 소스가 아니기 때문에 사용에 주의 하시길 바랍니다.",
                                            "../assets/offroad/icon_openpilot.png"
                                              ));
  toggles_list->addWidget(horizontal_line());
  toggles_list->addWidget(new ParamsToggle("롱컨트롤 사용",
                                            "N 롱컨트롤 기능 사용",
                                            "오픈파일럿이 속도를 조절합니다. 주의 하시길 바랍니다.",
                                            "../assets/offroad/icon_road.png"
                                              ));
  toggles_list->addWidget(horizontal_line());
  toggles_list->addWidget(new ParamsToggle("매드모드 사용",
                                            "HKG 매드모드 사용",
                                            "가감속의 사용 하지 않아도 핸들 조향을 사용합니다.",
                                            "../assets/offroad/icon_openpilot.png"
                                              ));
  toggles_list->addWidget(horizontal_line());
  toggles_list->addWidget(new ParamsToggle("자동 차선변경 사용",
                                            "자동 차선 변경",
                                            "사용에 주의 하십시오",
                                            "../assets/offroad/icon_road.png"
                                              ));
  toggles_list->addWidget(horizontal_line());
  toggles_list->addWidget(new ParamsToggle("가감속 스무서 사용",
                                            "N Smoother",
                                            "순정 ASCC 기능을 이용하여 가속과 감속을 부드럽게 할 수 있도록 도와 줍니다. 사용법을 정확히 인지하고 사용하십시오.",
                                            "../assets/offroad/icon_road.png"
                                            ));
  toggles_list->addWidget(horizontal_line());
  toggles_list->addWidget(new ParamsToggle("커브 감속 사용",
                                            "곡률에 따른 속도 감속 기능을 사용",
                                            "",
                                            "../assets/offroad/icon_road.png"
                                            ));
  toggles_list->addWidget(horizontal_line());
  toggles_list->addWidget(new ParamsToggle("크루즈 속도의 동기화",
                                            "크루즈 속도를 설정 후 엑셀로 인해 설정 속도보다 가속 속도가 높아지면 그 속도에 크루즈 설정 속도가 동기화 됩니다.",
                                            "",
                                            "../assets/offroad/icon_road.png"
                                            ));
  toggles_list->addWidget(horizontal_line());
  toggles_list->addWidget(new ParamsToggle("스무스 설정 버튼 변경",
                                            "크루즈 갭 설정 버튼으로 순정 ASCC와 가감속 스무서 설정을 변경합니다.",
                                            "",
                                            "../assets/offroad/icon_road.png"
                                            ));
                                            toggles_list->addWidget(horizontal_line());
  toggles_list->addWidget(new ParamsToggle("디버그 내용 보기",
                                            "가감속 등 디버그 내용을 화면에 띄웁니다.",
                                            "",
                                            "../assets/offroad/icon_shell.png"
                                            ));

  QWidget *widget = new QWidget;
  widget->setLayout(toggles_list);
  return widget;
}



QWidget * device_panel() {

  QVBoxLayout *device_layout = new QVBoxLayout;
  device_layout->setMargin(100);
  device_layout->setSpacing(30);

  Params params = Params();
  std::vector<std::pair<std::string, std::string>> labels = {
    {"Dongle ID", params.get("DongleId", false)},
  };

  // get serial number
  //std::string cmdline = util::read_file("/proc/cmdline");
  //auto delim = cmdline.find("serialno=");
  //if (delim != std::string::npos) {
  //  labels.push_back({"Serial", cmdline.substr(delim, cmdline.find(" ", delim))});
  //}

  for (auto &l : labels) {
    device_layout->addWidget(labelWidget(QString::fromStdString(l.first), QString::fromStdString(l.second)), 0, Qt::AlignTop);
  }

  QPushButton* dcam_view = new QPushButton("운전자 영상 미리보기");
  device_layout->addWidget(dcam_view, 0, Qt::AlignBottom);
  device_layout->addWidget(horizontal_line(), Qt::AlignBottom);
  QObject::connect(dcam_view, &QPushButton::released, [=]() {
    Params().write_db_value("IsDriverViewEnabled", "1", 1);
  });

  // TODO: show current calibration values
  QPushButton *clear_cal_btn = new QPushButton("캘리브레이션 리셋");
  device_layout->addWidget(clear_cal_btn, 0, Qt::AlignBottom);
  device_layout->addWidget(horizontal_line(), Qt::AlignBottom);
  QObject::connect(clear_cal_btn, &QPushButton::released, [=]() {
    if (ConfirmationDialog::confirm("캘리브레이션을 초기화합니다. 진행하시겠습니까?")) {
      Params().delete_db_value("CalibrationParams");
    }
  });

  // power buttons

  QPushButton *poweroff_btn = new QPushButton("전원 끄기");
  device_layout->addWidget(poweroff_btn, Qt::AlignBottom);
  QObject::connect(poweroff_btn, &QPushButton::released, [=]() {
    if (ConfirmationDialog::confirm("장치의 전원을 끕니다. 진행하시겠습니까?")) {
      Hardware::poweroff();
    }
  });

  device_layout->addWidget(horizontal_line(), Qt::AlignBottom);

  QPushButton *reboot_btn = new QPushButton("재부팅");
  device_layout->addWidget(reboot_btn, Qt::AlignBottom);
  QObject::connect(reboot_btn, &QPushButton::released, [=]() {
    if (ConfirmationDialog::confirm("장치를 재부팅 합니다. 진행하시겠습니까?")) {
      Hardware::reboot();
    }
  });

  QPushButton *uninstall_btn = new QPushButton("오픈파일럿 제거");
  device_layout->addWidget(uninstall_btn);
  QObject::connect(uninstall_btn, &QPushButton::released, [=]() {
    if (ConfirmationDialog::confirm("오픈파일럿을 제거합니다. 진행하시겠습니까?")) {
      Params().write_db_value("DoUninstall", "1");
    }
  });

  QWidget *widget = new QWidget;
  widget->setLayout(device_layout);
  widget->setStyleSheet(R"(
    QPushButton {
      padding: 0;
      height: 120px;
      border-radius: 15px;
      background-color: #393939;
    }
  )");
  return widget;
}

QWidget * developer_panel() {
  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->setMargin(100);

  Params params = Params();
  std::string brand = params.read_db_bool("Passive") ? "dashcam" : "openpilot";
  std::vector<std::pair<std::string, std::string>> labels = {
    {"Version", brand + " v" + params.get("Version", false)},
    {"Git Branch", params.get("GitBranch", false)},
    {"Git Commit", params.get("GitCommit", false).substr(0, 10)},
    {"Panda Firmware", params.get("PandaFirmwareHex", false)},
    {"OS Version", Hardware::get_os_version()},
  };

  for (int i = 0; i < labels.size(); i++) {
    auto l = labels[i];
    main_layout->addWidget(labelWidget(QString::fromStdString(l.first), QString::fromStdString(l.second)));

    if(i+1 < labels.size()) {
      main_layout->addWidget(horizontal_line());
    }
  }

  QWidget *widget = new QWidget;
  widget->setLayout(main_layout);
  widget->setStyleSheet(R"(
    QLabel {
      font-size: 50px;
    }
  )");
  return widget;
}

QWidget * network_panel(QWidget * parent) {
#ifdef QCOM
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(100);
  layout->setSpacing(30);

  // TODO: can probably use the ndk for this
  // simple wifi + tethering buttons
  std::vector<std::pair<const char*, const char*>> btns = {
    {"네트워크 설정열기", "am start -n com.android.settings/.wifi.WifiPickerActivity \
                            -a android.net.wifi.PICK_WIFI_NETWORK \
                            --ez extra_prefs_show_button_bar true \
                            --es extra_prefs_set_next_text ''"},
    {"테터링 설정열기", "am start -n com.android.settings/.TetherSettings \
                                 --ez extra_prefs_show_button_bar true \
                                 --es extra_prefs_set_next_text ''"},
  };
  for (auto &b : btns) {
    QPushButton *btn = new QPushButton(b.first);
    layout->addWidget(btn, 0, Qt::AlignTop);
    QObject::connect(btn, &QPushButton::released, [=]() { std::system(b.second); });
  }
  layout->addStretch(1);

  QWidget *w = new QWidget;
  w->setLayout(layout);
  w->setStyleSheet(R"(
    QPushButton {
      padding: 0;
      height: 120px;
      border-radius: 15px;
      background-color: #393939;
    }
  )");

#else
  Networking *w = new Networking(parent);
#endif
  return w;
}


void SettingsWindow::setActivePanel() {
  auto *btn = qobject_cast<QPushButton *>(nav_btns->checkedButton());
  panel_layout->setCurrentWidget(panels[btn->text()]);
}

SettingsWindow::SettingsWindow(QWidget *parent) : QFrame(parent) {
  // setup two main layouts
  QVBoxLayout *sidebar_layout = new QVBoxLayout();
  sidebar_layout->setMargin(0);
  panel_layout = new QStackedLayout();

  // close button
  QPushButton *close_btn = new QPushButton("X");
  close_btn->setStyleSheet(R"(
    font-size: 90px;
    font-weight: bold;
    border 1px grey solid;
    border-radius: 100px;
    background-color: #292929;
  )");
  close_btn->setFixedSize(200, 200);
  sidebar_layout->addSpacing(45);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignCenter);
  QObject::connect(close_btn, SIGNAL(released()), this, SIGNAL(closeSettings()));

  // setup panels

  std::vector<QString> panels_key = {
    "Developer",
    "Device",
    "Network",
    "Toggles",
    "Community",
  };

  panels = {
    {"개발자", developer_panel()},
    {"장치", device_panel()},
    {"네트워크", network_panel(this)},
    {"토글메뉴", toggles_panel()},
    {"커뮤니티", community_panel()},
  };

  sidebar_layout->addSpacing(45);
  nav_btns = new QButtonGroup();
  for (auto key : panels_key) {
    auto& panel = *(panels.find(key));
    QPushButton *btn = new QPushButton(panel.first);
    btn->setCheckable(true);
    btn->setStyleSheet(R"(
      * {
        color: grey;
        border: none;
        background: none;
        font-size: 65px;
        font-weight: 500;
        padding-top: 35px;
        padding-bottom: 35px;
      }
      QPushButton:checked {
        color: white;
      }
    )");

    nav_btns->addButton(btn);
    sidebar_layout->addWidget(btn, 0, Qt::AlignRight);
    panel_layout->addWidget(panel.second);
    QObject::connect(btn, SIGNAL(released()), this, SLOT(setActivePanel()));
    QObject::connect(btn, &QPushButton::released, [=](){emit sidebarPressed();});
  }
  qobject_cast<QPushButton *>(nav_btns->buttons()[0])->setChecked(true);
  sidebar_layout->setContentsMargins(50, 50, 100, 50);

  // main settings layout, sidebar + main panel
  QHBoxLayout *settings_layout = new QHBoxLayout();

  sidebar_widget = new QWidget;
  sidebar_widget->setLayout(sidebar_layout);
  sidebar_widget->setFixedWidth(500);
  settings_layout->addWidget(sidebar_widget);


  panel_frame = new QFrame;
  panel_frame->setLayout(panel_layout);
  panel_frame->setStyleSheet(R"(
    QFrame {
      border-radius: 30px;
      background-color: #292929;
    }
    * {
      background-color: none;
    }
  )");
  settings_layout->addWidget(panel_frame);

  setLayout(settings_layout);
  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    SettingsWindow {
      background-color: black;
    }
  )");
}
