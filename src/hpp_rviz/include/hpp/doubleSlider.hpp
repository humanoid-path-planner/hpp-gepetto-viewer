
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QWidget>

class DoubleSlider : public QWidget {
  Q_OBJECT
 public:
  DoubleSlider(double min, double max, double step, QWidget* parent = nullptr)
      : QWidget(parent), step_(step) {
    int steps = static_cast<int>((max - min) / step);
    slider_ = new QSlider(Qt::Horizontal);
    slider_->setRange(0, steps);
    spinbox_ = new QDoubleSpinBox();
    spinbox_->setRange(min, max);
    spinbox_->setSingleStep(step);
    spinbox_->setDecimals(decimalsForStep(step));

    auto* l = new QHBoxLayout(this);
    l->addWidget(slider_);
    l->addWidget(spinbox_);

    connect(slider_, &QSlider::valueChanged, this, [=](int v) {
      double val = min + v * step_;
      spinbox_->blockSignals(true);
      spinbox_->setValue(val);
      spinbox_->blockSignals(false);
      emit valueChanged(val);
    });
    connect(spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [=](double val) {
              slider_->blockSignals(true);
              slider_->setValue(static_cast<int>((val - min) / step_));
              slider_->blockSignals(false);
              emit valueChanged(val);
            });
  }
  void setRange(double min, double max) {
    int steps = static_cast<int>((max - min) / step_);
    slider_->setRange(0, steps);
    spinbox_->setRange(min, max);
  }

  double value() const { return spinbox_->value(); }
  void setValue(double v) { spinbox_->setValue(v); }

 signals:
  void valueChanged(double value);

 private:
  QSlider* slider_;
  QDoubleSpinBox* spinbox_;
  double step_;

  int decimalsForStep(double s) {
    int d = 0;
    while (s < 1.0 && d < 6) {
      s *= 10;
      d++;
    }
    return d;
  }
};
