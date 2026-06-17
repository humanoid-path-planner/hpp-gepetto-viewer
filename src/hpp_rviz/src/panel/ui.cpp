#include "../../include/hpp/panel/trajectory_slider.hpp"

namespace hpp {
namespace panel {

void TrajectorySlider::initializeUi() {
  const auto layout = new QVBoxLayout(this);
  layout->setAlignment(Qt::AlignTop);

  info_label_ = new QLabel("Waiting for a path...", this);
  layout->addWidget(info_label_);

  // Slider
  slider_ = new QSlider(Qt::Horizontal, this);
  slider_->setMinimum(0);
  slider_->setMaximum(1000);
  slider_->setValue(0);
  slider_->setEnabled(false);
  layout->addWidget(slider_);

  // Time label
  time_label_ = new QLabel("t = 0.000 s / 0.000 s", this);
  time_label_->setAlignment(Qt::AlignCenter);
  layout->addWidget(time_label_);

  // Bouton Play/Pause
  toogle_play_button_ = new QPushButton("▶ Play", this);
  toogle_play_button_->setEnabled(false);
  layout->addWidget(toogle_play_button_);

  // t
  t_layout_ = new QHBoxLayout();
  t_label_ = new QLabel("t :", this);
  t_spinbox_ = new QDoubleSpinBox(this);
  t_spinbox_->setMinimum(0.001);
  t_spinbox_->setMaximum(10000.001);
  t_spinbox_->setSingleStep(0.01);
  t_spinbox_->setValue(0.000);
  t_spinbox_->setSuffix(" s");
  t_layout_->addWidget(t_label_);
  t_layout_->addWidget(t_spinbox_);
  layout->addLayout(t_layout_);

  // Vitesse
  speed_layout_ = new QHBoxLayout();
  speed_label_ = new QLabel("Speed :", this);
  speed_spinbox_ = new QDoubleSpinBox(this);
  speed_spinbox_->setMinimum(0.01);
  speed_spinbox_->setMaximum(10.0);
  speed_spinbox_->setSingleStep(0.1);
  speed_spinbox_->setValue(1.0);
  speed_spinbox_->setSuffix(" x");
  speed_layout_->addWidget(speed_label_);
  speed_layout_->addWidget(speed_spinbox_);
  layout->addLayout(speed_layout_);

  // Target frame selector (placeholder)
  target_frame_label_ = new QLabel("Target frame:", this);
  target_frame_selector_ = new QComboBox(this);
  target_frame_layout_ = new QHBoxLayout();
  target_frame_layout_->addWidget(target_frame_label_);
  target_frame_layout_->addWidget(target_frame_selector_);
  layout->addLayout(target_frame_layout_);

  play_timer_ = new QTimer(this);
  play_timer_->setInterval(static_cast<int>(1000 / trajectoryFps));

  auto* hpp_vec_header = new QHBoxLayout();
  auto* hpp_vec_title = new QLabel("HppVectorConfiguration:", this);
  hpp_vec_header->addWidget(hpp_vec_title);
  hpp_vec_header->addStretch();
  copy_button_ = new QPushButton("Copy", this);
  copy_button_->setFixedWidth(80);
  hpp_vec_header->addWidget(copy_button_);
  layout->addLayout(hpp_vec_header);

  hpp_vector_configuration_edit_ = new QPlainTextEdit(this);
  hpp_vector_configuration_edit_->setReadOnly(true);
  hpp_vector_configuration_edit_->setLineWrapMode(QPlainTextEdit::WidgetWidth);
  hpp_vector_configuration_edit_->setMaximumHeight(80);
  hpp_vector_configuration_edit_->setSizePolicy(QSizePolicy::Expanding,
                                                QSizePolicy::Fixed);
  hpp_vector_configuration_edit_->setPlaceholderText("No configuration yet...");
  layout->addWidget(hpp_vector_configuration_edit_);

  /////TREEEE
  tree_ = new QTreeWidget(this);

  tree_->setColumnCount(2);
  tree_->setHeaderHidden(false);
  tree_->setHeaderLabels({"Name", "Value"});
  tree_->header()->setSectionResizeMode(
      0, QHeaderView::Interactive);  // ← draggable
  tree_->header()->setSectionResizeMode(1, QHeaderView::Stretch);
  tree_->header()->setStretchLastSection(true);

  tree_->setColumnWidth(0, 200);

  tree_->setFrameShape(QFrame::NoFrame);
  tree_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  tree_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  tree_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  layout->addWidget(tree_);

  setLayout(layout);

  grpJoints_ = new QTreeWidgetItem(tree_, {"Actual Position"});
  grpJoints_->setExpanded(true);
}

double TrajectorySlider::computeStep(double min, double max) {
  double range = std::abs(max - min);
  if (range == 0.0) return 0.01;

  // Vise ~200 steps pour une bonne précision
  double raw_step = range / 200.0;

  // Arrondit au multiple de 10 le plus proche (0.001, 0.01, 0.1, 1.0, ...)
  double magnitude = std::pow(10.0, std::floor(std::log10(raw_step)));
  double normalized = raw_step / magnitude;

  double step;
  if (normalized < 2.0)
    step = magnitude;
  else if (normalized < 5.0)
    step = 2.0 * magnitude;
  else
    step = 5.0 * magnitude;

  return step;
}

QTreeWidgetItem* TrajectorySlider::createSliderTreeItem(QTreeWidgetItem* parent,
                                                        const QString& label,
                                                        double min, double max,
                                                        double defaultVal) {
  auto* item = new QTreeWidgetItem(parent, {label});
  item->setSizeHint(0, QSize(0, 20));
  auto* my_slider = new DoubleSlider(min, max, computeStep(min, max), this);
  my_slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  item->setData(1, Qt::UserRole, QVariant::fromValue(my_slider));
  my_slider->blockSignals(true);
  my_slider->setValue(defaultVal);
  my_slider->blockSignals(false);
  return item;
}

QTreeWidgetItem* TrajectorySlider::addJointSliderItem(QTreeWidgetItem* parent,
                                                      const std::string& name,
                                                      const QString& label,
                                                      double min, double max,
                                                      double defaultVal) {
  QTreeWidgetItem* item =
      createSliderTreeItem(parent, label, min, max, defaultVal);
  auto* container = new QWidget;
  container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  container->setContentsMargins(0, 0, 0, 0);
  auto* hbox = new QHBoxLayout(container);
  hbox->setContentsMargins(0, 0, 0, 0);
  hbox->setSpacing(0);
  hbox->setMargin(0);

  DoubleSlider* my_slider = item->data(1, Qt::UserRole).value<DoubleSlider*>();
  hbox->addWidget(my_slider);
  tree_->setItemWidget(item, 1, container);
  connect(my_slider, QOverload<double>::of(&DoubleSlider::valueChanged),
          [this, name](double v) { onJointValueChanged(name, v); });
  item->setData(1, Qt::UserRole, QVariant::fromValue(my_slider));
  return item;
}

void TrajectorySlider::addFreeFlyerSliderItem(
    QTreeWidgetItem* parent, const std::string& name, double min, double max,
    const std::vector<double>& defaultVal) {
  std::vector<std::string> labels{"x", "y", "z", "qx", "qy", "qz", "qw"};

  for (int i = 0; i < 7; ++i) {
    if (i >= 3) {
      min = -1.0;
      max = 1.0;
    }
    QTreeWidgetItem* item = createSliderTreeItem(parent, labels[i].c_str(), min,
                                                 max, defaultVal[i]);
    auto* container = new QWidget;
    container->setContentsMargins(0, 0, 0, 0);
    auto* hbox = new QHBoxLayout(container);
    hbox->setContentsMargins(0, 0, 0, 0);
    hbox->setMargin(0);
    hbox->setSpacing(0);

    DoubleSlider* my_slider =
        item->data(1, Qt::UserRole).value<DoubleSlider*>();

    hbox->addWidget(my_slider);
    tree_->setItemWidget(item, 1, container);
    freeflyerValues_[name][i] = defaultVal[i];

    connect(my_slider, QOverload<double>::of(&DoubleSlider::valueChanged),
            [this, name, i](double v) { onFreeFlyerValueChanged(name, v, i); });

    item->setData(1, Qt::UserRole, QVariant::fromValue(my_slider));
  }
}

}  // namespace panel
}  // namespace hpp
