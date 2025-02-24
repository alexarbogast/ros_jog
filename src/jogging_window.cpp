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

#include <QApplication>
#include <QBoxLayout>

#include <ros_jog/jogging_window.h>


namespace ros_jog
{
JoggingWindow::JoggingWindow(QWidget* parent) : QMainWindow(parent)
{
  QWidget* central_widget = new QWidget;
  QHBoxLayout* layout = new QHBoxLayout(central_widget);
  setCentralWidget(central_widget);

  cartesian_jogger_ = new CartesianJogger(this);
  layout->addWidget(cartesian_jogger_);
}
}  // namespace ros_jog

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  ros_jog::JoggingWindow window;
  window.show();
  return app.exec();
}
