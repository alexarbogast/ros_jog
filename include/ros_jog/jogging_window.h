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

#pragma once

#include <QMainWindow>

#include <ros_jog/cartesian_jog_widget.h>

namespace ros_jog
{
class JoggingWindow : public QMainWindow
{
  Q_OBJECT
public:
  explicit JoggingWindow(QWidget *parent = 0);
private:
  CartesianJogger* cartesian_jogger_;
};

}  // namespace ros_jog
