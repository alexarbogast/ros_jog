// Copyright 2025 Alex Arbogast
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <qboxlayout.h>
#include <ros_jog/cartesian_jog_widget.h>

#include <QGridLayout>
#include <QLabel>

namespace ros_jog
{
AxisButton::AxisButton(const QString& text, QWidget* parent)
  : QToolButton(parent)
{
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  setText(text);
}

QSize AxisButton::sizeHint() const
{
  QSize size = QToolButton::sizeHint();
  size.rheight() += 20;
  size.rwidth() = qMax(size.width(), size.height());
  return size;
}

CartesianJogger::CartesianJogger(QWidget* parent) : QWidget(parent)
{
  QVBoxLayout* main_layout = new QVBoxLayout;

  QLabel* title = new QLabel("Cartesian Jogging");
  title->setStyleSheet("font-weight: bold; font-size: 16px;");
  title->setAlignment(Qt::AlignCenter);
  title->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  main_layout->addWidget(title);

  QGridLayout* layout = new QGridLayout;
  AxisButton* x_pos_button = new AxisButton("X+");
  AxisButton* y_pos_button = new AxisButton("Y+");
  AxisButton* z_pos_button = new AxisButton("Z+");
  AxisButton* r_pos_button = new AxisButton("R+");
  AxisButton* p_pos_button = new AxisButton("P+");
  AxisButton* w_pos_button = new AxisButton("W+");
  AxisButton* x_neg_button = new AxisButton("X-");
  AxisButton* y_neg_button = new AxisButton("Y-");
  AxisButton* z_neg_button = new AxisButton("Z-");
  AxisButton* r_neg_button = new AxisButton("R-");
  AxisButton* p_neg_button = new AxisButton("P-");
  AxisButton* w_neg_button = new AxisButton("W-");


  layout->addWidget(x_pos_button, 1, 2);
  layout->addWidget(y_pos_button, 0, 1);
  layout->addWidget(z_pos_button, 0, 4);
  layout->addWidget(r_pos_button, 0, 5);
  layout->addWidget(p_pos_button, 0, 6);
  layout->addWidget(w_pos_button, 0, 7);
  layout->addWidget(x_neg_button, 1, 0);
  layout->addWidget(y_neg_button, 2, 1);
  layout->addWidget(z_neg_button, 2, 4);
  layout->addWidget(r_neg_button, 2, 5);
  layout->addWidget(p_neg_button, 2, 6);
  layout->addWidget(w_neg_button, 2, 7);

  main_layout->addLayout(layout);
  setLayout(main_layout);
}
}  // namespace ros_jog
