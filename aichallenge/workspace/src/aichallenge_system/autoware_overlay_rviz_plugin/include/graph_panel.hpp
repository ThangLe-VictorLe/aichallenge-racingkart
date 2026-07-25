#ifndef GRAPH_PANEL_HPP_
#define GRAPH_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <QColor>
#include <QLabel>
#include <QMutex>
#include <QPaintEvent>
#include <QPainter>
#include <QTableWidget>
#include <QHeaderView>
#include <QTimer>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QWidget>

#include <deque>
#include <memory>
#include <mutex>
#endif

namespace aichallenge_rviz_plugin
{

// ─── Custom scrolling graph widget ─────────────────────────────────────────
class GraphWidget : public QWidget
{
  Q_OBJECT
public:
  explicit GraphWidget(QWidget * parent = nullptr);

  // Thread-safe setters called from ROS callbacks
  void pushSpeed(double value);
  void pushAccel(double value);
  void pushBrake(double value);

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  static constexpr int kMaxPoints = 300;

  // Rolling buffers (values in [0,1] for accel/brake; km/h for speed)
  std::deque<double> speed_buf_;
  std::deque<double> accel_buf_;
  std::deque<double> brake_buf_;

  QMutex mutex_;

  // Drawing helpers
  void drawGrid(QPainter & p, const QRectF & rect);
  void drawSeries(QPainter & p, const QRectF & rect,
                  const std::deque<double> & buf, double y_min, double y_max,
                  const QColor & color, const QString & label, double current);
  void drawLegend(QPainter & p, const QRectF & rect);
};

// ─── Panel ──────────────────────────────────────────────────────────────────
class GraphPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit GraphPanel(QWidget * parent = nullptr);
  ~GraphPanel() override;

  void onInitialize() override;

Q_SIGNALS:
  void lapCompleted(int lap, double time_sec, double avg_speed_kmh);
  void lapReset();

private Q_SLOTS:
  void onLapCompleted(int lap, double time_sec, double avg_speed_kmh);
  void onLapReset();

private:
  // ROS subscriptions
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr speed_sub_;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr status_sub_;

  // Qt widgets
  GraphWidget * graph_widget_{nullptr};
  QLabel * status_label_{nullptr};
  QTableWidget * lap_table_{nullptr};
  QTimer * repaint_timer_{nullptr};

  // State
  int current_lap_{-1};
  double last_awsim_lap_time_{0.0};  // data[2] from previous message
  double current_lap_speed_sum_{0.0};
  int current_lap_speed_count_{0};

  // Shared data (written by ROS thread, read by Qt thread via GraphWidget's mutex)
  std::mutex data_mutex_;
};

}  // namespace aichallenge_rviz_plugin

#endif  // GRAPH_PANEL_HPP_
