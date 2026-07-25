#include "graph_panel.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <QFont>
#include <QPainterPath>
#include <QPen>
#include <QString>

#include <algorithm>
#include <cmath>
#include <string>

namespace aichallenge_rviz_plugin
{

// ════════════════════════════════════════════════════════════════════════════
//  GraphWidget
// ════════════════════════════════════════════════════════════════════════════

GraphWidget::GraphWidget(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(300, 220);
  // Dark background
  QPalette pal = palette();
  pal.setColor(QPalette::Window, QColor(18, 18, 28));
  setAutoFillBackground(true);
  setPalette(pal);
}

void GraphWidget::pushSpeed(double value)
{
  QMutexLocker lock(&mutex_);
  speed_buf_.push_back(value);
  if (static_cast<int>(speed_buf_.size()) > kMaxPoints) {
    speed_buf_.pop_front();
  }
}

void GraphWidget::pushAccel(double value)
{
  QMutexLocker lock(&mutex_);
  accel_buf_.push_back(value);
  if (static_cast<int>(accel_buf_.size()) > kMaxPoints) {
    accel_buf_.pop_front();
  }
}

void GraphWidget::pushBrake(double value)
{
  QMutexLocker lock(&mutex_);
  brake_buf_.push_back(value);
  if (static_cast<int>(brake_buf_.size()) > kMaxPoints) {
    brake_buf_.pop_front();
  }
}

// ─── paintEvent ─────────────────────────────────────────────────────────────
void GraphWidget::paintEvent(QPaintEvent * /*event*/)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  const QRectF full(0, 0, width(), height());

  // Background
  p.fillRect(full, QColor(18, 18, 28));

  // Title
  QFont titleFont("monospace", 9, QFont::Bold);
  p.setFont(titleFont);
  p.setPen(QColor(200, 200, 220));
  p.drawText(QRectF(0, 4, width(), 20), Qt::AlignHCenter, "Accel / Speed / Brake");

  const int margin_top = 28;
  const int margin_bottom = 30;
  const int margin_left = 8;
  const int margin_right = 8;
  const int legend_h = 16;
  const int graph_area_h =
    height() - margin_top - margin_bottom - legend_h;
  const int graph_h = graph_area_h / 3;

  // Lock data
  QMutexLocker lock(&mutex_);

  // ── SPEED (km/h) ──
  {
    QRectF rect(margin_left, margin_top, width() - margin_left - margin_right, graph_h);
    double cur = speed_buf_.empty() ? 0.0 : speed_buf_.back();
    drawGrid(p, rect);
    drawSeries(p, rect, speed_buf_, 0.0, 80.0,
               QColor(80, 200, 255), "Speed (km/h)", cur);
  }

  // ── ACCEL ──
  {
    QRectF rect(margin_left, margin_top + graph_h, width() - margin_left - margin_right, graph_h);
    double cur = accel_buf_.empty() ? 0.0 : accel_buf_.back();
    drawGrid(p, rect);
    drawSeries(p, rect, accel_buf_, 0.0, 1.0,
               QColor(100, 230, 120), "Accel cmd", cur);
  }

  // ── BRAKE ──
  {
    QRectF rect(margin_left, margin_top + graph_h * 2,
                width() - margin_left - margin_right, graph_h);
    double cur = brake_buf_.empty() ? 0.0 : brake_buf_.back();
    drawGrid(p, rect);
    drawSeries(p, rect, brake_buf_, 0.0, 1.0,
               QColor(255, 90, 90), "Brake cmd", cur);
  }

  // Legend at the bottom
  QRectF legendRect(margin_left, height() - legend_h - 4,
                    width() - margin_left - margin_right, legend_h);
  drawLegend(p, legendRect);
}

// ─── drawGrid ───────────────────────────────────────────────────────────────
void GraphWidget::drawGrid(QPainter & p, const QRectF & rect)
{
  p.setPen(QPen(QColor(50, 50, 70), 1, Qt::DotLine));
  // 4 horizontal lines
  for (int i = 1; i < 4; ++i) {
    double y = rect.top() + rect.height() * i / 4.0;
    p.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y));
  }
  // Thin border
  p.setPen(QPen(QColor(60, 60, 80), 1));
  p.drawRect(rect);
}

// ─── drawSeries ─────────────────────────────────────────────────────────────
void GraphWidget::drawSeries(QPainter & p, const QRectF & rect,
                              const std::deque<double> & buf,
                              double y_min, double y_max,
                              const QColor & color, const QString & label, double current)
{
  const int n = static_cast<int>(buf.size());

  // Draw a filled area under the line
  if (n >= 2) {
    QPainterPath path;
    double range = y_max - y_min;
    if (range < 1e-9) { range = 1.0; }

    auto to_x = [&](int i) -> double {
      return rect.left() + rect.width() * i / (kMaxPoints - 1.0);
    };
    auto to_y = [&](double v) -> double {
      double norm = std::min(std::max((v - y_min) / range, 0.0), 1.0);
      return rect.bottom() - norm * rect.height();
    };

    // Offset: draw from right-aligned position
    int offset = kMaxPoints - n;

    path.moveTo(to_x(offset), rect.bottom());
    for (int i = 0; i < n; ++i) {
      path.lineTo(to_x(offset + i), to_y(buf[i]));
    }
    path.lineTo(to_x(offset + n - 1), rect.bottom());
    path.closeSubpath();

    QColor fill_color = color;
    fill_color.setAlpha(50);
    p.fillPath(path, fill_color);

    // Line
    QPainterPath line_path;
    line_path.moveTo(to_x(offset), to_y(buf[0]));
    for (int i = 1; i < n; ++i) {
      line_path.lineTo(to_x(offset + i), to_y(buf[i]));
    }
    p.setPen(QPen(color, 1.8));
    p.drawPath(line_path);
  }

  // Label on left
  QFont lf("monospace", 7);
  p.setFont(lf);
  p.setPen(color);
  p.drawText(
    QRectF(rect.left() + 3, rect.top() + 2, 100, 14),
    Qt::AlignLeft | Qt::AlignVCenter, label);

  // Current value on right
  QString cur_str = QString::number(current, 'f', 2);
  p.drawText(
    QRectF(rect.right() - 55, rect.top() + 2, 52, 14),
    Qt::AlignRight | Qt::AlignVCenter, cur_str);
}

// ─── drawLegend ─────────────────────────────────────────────────────────────
void GraphWidget::drawLegend(QPainter & p, const QRectF & rect)
{
  struct Entry { QColor color; QString label; };
  const Entry entries[] = {
    {QColor(80,  200, 255), "Speed km/h"},
    {QColor(100, 230, 120), "Accel [0-1]"},
    {QColor(255,  90,  90), "Brake [0-1]"},
  };

  QFont lf("monospace", 7);
  p.setFont(lf);

  double x = rect.left();
  double entry_w = rect.width() / 3.0;
  for (const auto & e : entries) {
    // Color swatch
    p.fillRect(QRectF(x + 4, rect.top() + 4, 10, 8), e.color);
    // Text
    p.setPen(QColor(180, 180, 200));
    p.drawText(QRectF(x + 18, rect.top(), entry_w - 18, rect.height()),
               Qt::AlignLeft | Qt::AlignVCenter, e.label);
    x += entry_w;
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  GraphPanel
// ════════════════════════════════════════════════════════════════════════════

GraphPanel::GraphPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  graph_widget_ = new GraphWidget(this);

  status_label_ = new QLabel("Waiting for data…", this);
  status_label_->setAlignment(Qt::AlignHCenter);
  QPalette lp = status_label_->palette();
  lp.setColor(QPalette::WindowText, QColor(160, 160, 180));
  status_label_->setPalette(lp);
  QFont sf("monospace", 8);
  status_label_->setFont(sf);

  // Table setup
  lap_table_ = new QTableWidget(this);
  lap_table_->setColumnCount(3);
  lap_table_->setHorizontalHeaderLabels({"Lap", "Time (s)", "Avg Speed (km/h)"});
  lap_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  lap_table_->verticalHeader()->setVisible(false);
  lap_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  lap_table_->setSelectionMode(QAbstractItemView::NoSelection);
  lap_table_->setStyleSheet(
    "QTableWidget { background-color: #12121c; color: #dcdcdc; gridline-color: #323246; }"
    "QHeaderView::section { background-color: #1c1c28; color: #dcdcdc; }");

  auto * left_layout = new QVBoxLayout();
  left_layout->setContentsMargins(0, 0, 0, 0);
  left_layout->setSpacing(2);
  left_layout->addWidget(graph_widget_, 1);
  left_layout->addWidget(status_label_);

  auto * main_layout = new QHBoxLayout(this);
  main_layout->setContentsMargins(4, 4, 4, 4);
  main_layout->setSpacing(8);
  main_layout->addLayout(left_layout, 2); // graph takes more space
  main_layout->addWidget(lap_table_, 1);  // table takes less
  setLayout(main_layout);

  // Connect signals/slots for thread-safe UI updates
  connect(this, &GraphPanel::lapCompleted, this, &GraphPanel::onLapCompleted, Qt::QueuedConnection);
  connect(this, &GraphPanel::lapReset, this, &GraphPanel::onLapReset, Qt::QueuedConnection);

  // Repaint the graph at ~30 Hz regardless of topic rate
  repaint_timer_ = new QTimer(this);
  connect(repaint_timer_, &QTimer::timeout, graph_widget_,
          static_cast<void (QWidget::*)()>(&QWidget::update));
  repaint_timer_->start(33);  // ~30 fps
}

GraphPanel::~GraphPanel()
{
  if (repaint_timer_) {
    repaint_timer_->stop();
  }
  speed_sub_.reset();
  control_sub_.reset();
  status_sub_.reset();
}

void GraphPanel::onInitialize()
{
  rviz_common::Panel::onInitialize();

  auto rviz_ros_node = getDisplayContext()->getRosNodeAbstraction().lock();
  auto node = rviz_ros_node->get_raw_node();

  // ── Speed subscription ───────────────────────────────────────────────────
  speed_sub_ = node->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status",
    rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(),
    [this](const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
      double speed_kmh = std::abs(msg->longitudinal_velocity) * 3.6;
      graph_widget_->pushSpeed(speed_kmh);
      status_label_->setText(
        QString("Speed: %1 km/h").arg(speed_kmh, 5, 'f', 1));
        
      std::lock_guard<std::mutex> lock(data_mutex_);
      current_lap_speed_sum_ += speed_kmh;
      current_lap_speed_count_++;
    });

  // ── Control Command subscription ─────────────────────────────────────────
  control_sub_ = node->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd",
    rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(),
    [this](const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg) {
      double accel = msg->longitudinal.acceleration;
      if (accel >= 0.0) {
        graph_widget_->pushAccel(accel);
        graph_widget_->pushBrake(0.0);
      } else {
        graph_widget_->pushAccel(0.0);
        graph_widget_->pushBrake(-accel);
      }
    });

  // ── AWSIM Status subscription (Lap tracking) ─────────────────────────────
  // Topic: /awsim/status  (Float32MultiArray)
  //   data[0] = unused
  //   data[1] = laps completed (counts up each time a lap finishes)
  //   data[2] = rolling lap timer in seconds (resets at start of each lap)
  status_sub_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/awsim/status", 10,
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      if (msg->data.size() >= 3) {
        int lap = static_cast<int>(msg->data[1]);
        double awsim_lap_time = static_cast<double>(msg->data[2]);

        std::lock_guard<std::mutex> lock(data_mutex_);
        if (current_lap_ == -1) {
          // First message: initialise (lap counter starts at 0 before race begins)
          current_lap_ = (lap == 0) ? 1 : lap;
          last_awsim_lap_time_ = awsim_lap_time;
          current_lap_speed_sum_ = 0.0;
          current_lap_speed_count_ = 0;
        } else if (lap > current_lap_) {
          // A new lap has started -- the AWSIM timer just rolled over.
          // last_awsim_lap_time_ holds data[2] from the LAST message of the
          // previous lap, which is the authoritative lap time.
          double avg_speed = (current_lap_speed_count_ > 0)
            ? (current_lap_speed_sum_ / current_lap_speed_count_) : 0.0;

          Q_EMIT lapCompleted(current_lap_, last_awsim_lap_time_, avg_speed);

          current_lap_ = lap;
          current_lap_speed_sum_ = 0.0;
          current_lap_speed_count_ = 0;
        } else if (lap == 0 && current_lap_ > 1) {
          // Race reset
          current_lap_ = 1;
          current_lap_speed_sum_ = 0.0;
          current_lap_speed_count_ = 0;
          Q_EMIT lapReset();
        }
        last_awsim_lap_time_ = awsim_lap_time;
      }
    });
}

void GraphPanel::onLapCompleted(int lap, double time_sec, double avg_speed_kmh)
{
  int row = lap_table_->rowCount();
  lap_table_->insertRow(row);
  
  auto * item_lap = new QTableWidgetItem(QString::number(lap));
  item_lap->setTextAlignment(Qt::AlignCenter);
  lap_table_->setItem(row, 0, item_lap);

  auto * item_time = new QTableWidgetItem(QString::number(time_sec, 'f', 2));
  item_time->setTextAlignment(Qt::AlignCenter);
  lap_table_->setItem(row, 1, item_time);

  auto * item_speed = new QTableWidgetItem(QString::number(avg_speed_kmh, 'f', 1));
  item_speed->setTextAlignment(Qt::AlignCenter);
  lap_table_->setItem(row, 2, item_speed);
  
  lap_table_->scrollToBottom();
}

void GraphPanel::onLapReset()
{
  lap_table_->setRowCount(0);
}

}  // namespace aichallenge_rviz_plugin

PLUGINLIB_EXPORT_CLASS(aichallenge_rviz_plugin::GraphPanel, rviz_common::Panel)
