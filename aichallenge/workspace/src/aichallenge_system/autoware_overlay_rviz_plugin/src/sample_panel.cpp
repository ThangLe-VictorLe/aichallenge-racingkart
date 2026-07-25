#include <rviz_common/panel.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>

namespace aichallenge_rviz_plugin
{

class SamplePanel : public rviz_common::Panel
{
public:
  SamplePanel(QWidget * parent = nullptr) : rviz_common::Panel(parent)
    {
        // Create a label
        QLabel* label = new QLabel("Sample Text");

        // Create a layout
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addWidget(label);

        // Apply the layout to this panel
        setLayout(layout);
    }
};

}

// プラグインとして登録
PLUGINLIB_EXPORT_CLASS(aichallenge_rviz_plugin::SamplePanel, rviz_common::Panel)