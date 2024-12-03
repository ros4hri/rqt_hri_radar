// Copyright 2024 pal-robotics
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

#include <QDialog>
#include <QTableWidget>
#include <tuple>
#include <string>
#include <vector>

namespace rqt_human_radar
{

typedef std::tuple<std::string, std::string, std::string> Triple;

class SPOEditor : public QDialog
{
  Q_OBJECT

public:
  explicit SPOEditor(const std::string & subject = "", QWidget * parent = nullptr);

  void setTriples(const std::vector<Triple> & spo);

  std::vector<Triple> getTriples() const;

private slots:
  void addEmptyRow()
  {
    addRow();
  }
  void removeSelectedRow();

private:
  void addRow(
    const QString & subject = "", const QString & predicate = "",
    const QString & object = "");
  QTableWidget * tableWidget;

  std::string subject_;
};

}   // namespace rqt_human_radar
