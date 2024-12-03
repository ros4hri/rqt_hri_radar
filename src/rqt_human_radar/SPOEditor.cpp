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

#include <QDialog>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QHeaderView>
#include <QMessageBox>
#include <QComboBox>
#include <QStringList>

#include "rqt_human_radar/SPOEditor.hpp"

const QStringList PREDICATES {
  "dbp:color",
  "oro:isAt",
  "oro:isOn",
  "oro:contains",
  "dbp:name",
  "foaf:knows",
};

namespace rqt_human_radar
{
SPOEditor::SPOEditor(
  const std::string & subject, QWidget * parent
)
: QDialog(parent), subject_(subject)
{
  setWindowTitle("RDF/OWL triples");
  setFixedSize(500, 300);

  // Create the table widget
  tableWidget = new QTableWidget(this);
  tableWidget->setColumnCount(3);       // 3 columns for Subject, Predicate, Object
  tableWidget->setHorizontalHeaderLabels({"Subject", "Predicate", "Object"});
  tableWidget->horizontalHeader()->setStretchLastSection(true);       // Stretch last column
  tableWidget->verticalHeader()->setVisible(false);       // Hide row headers
  tableWidget->setEditTriggers(QAbstractItemView::DoubleClicked);       // Enable in-place editing

  QPushButton * addRowButton = new QPushButton("Add row", this);
  QPushButton * removeRowButton = new QPushButton("Remove selected row", this);

  connect(addRowButton, &QPushButton::clicked, this, &SPOEditor::addEmptyRow);
  connect(removeRowButton, &QPushButton::clicked, this, &SPOEditor::removeSelectedRow);

  QPushButton * okButton = new QPushButton("OK", this);
  QPushButton * cancelButton = new QPushButton("Cancel", this);

  connect(okButton, &QPushButton::clicked, this, &SPOEditor::accept);
  connect(cancelButton, &QPushButton::clicked, this, &SPOEditor::reject);

  // Layout the widgets
  QVBoxLayout * layout = new QVBoxLayout(this);
  layout->addWidget(tableWidget);

  // Layout for the Add/Remove buttons
  QHBoxLayout * rowButtonsLayout = new QHBoxLayout();
  rowButtonsLayout->addWidget(addRowButton);
  rowButtonsLayout->addWidget(removeRowButton);
  layout->addLayout(rowButtonsLayout);

  // Layout for the OK/Cancel buttons
  QHBoxLayout * okCancelLayout = new QHBoxLayout();
  okCancelLayout->addStretch();       // Push buttons to the right
  okCancelLayout->addWidget(okButton);
  okCancelLayout->addWidget(cancelButton);
  layout->addLayout(okCancelLayout);


  setLayout(layout);
}

void SPOEditor::setTriples(const std::vector<Triple> & spo)
{
  // Clear the table
  tableWidget->setRowCount(0);

  // Add the triples to the table
  for (const auto & triple : spo) {
    addRow(
      QString::fromStdString(std::get<0>(triple)),
      QString::fromStdString(std::get<1>(triple)),
      QString::fromStdString(std::get<2>(triple)));
  }
}

std::vector<Triple> SPOEditor::getTriples() const
{
  std::vector<Triple> spo;
  for (int row = 0; row < tableWidget->rowCount(); ++row) {
    std::string subject = tableWidget->item(row, 0)->text().toStdString();
    QComboBox * predicateComboBox = qobject_cast<QComboBox *>(tableWidget->cellWidget(row, 1));
    std::string predicate = predicateComboBox->currentText().toStdString();
    std::string object = tableWidget->item(row, 2)->text().toStdString();
    spo.push_back(std::make_tuple(subject, predicate, object));
  }
  return spo;
}

void SPOEditor::addRow(
  const QString & subject, const QString & predicate,
  const QString & object)
{
  QString final_subject = QString::fromStdString(subject_);
  if (!subject.isEmpty()) {
    final_subject = subject;
  }

  // Add an empty row to the table
  int newRow = tableWidget->rowCount();
  tableWidget->insertRow(newRow);

  // Optionally set placeholders for new cells
  tableWidget->setItem(newRow, 0, new QTableWidgetItem(final_subject));
  tableWidget->setItem(newRow, 2, new QTableWidgetItem(object));

  QComboBox * predicateComboBox = new QComboBox(this);
  predicateComboBox->setEditable(true);  // Allow user to type in new predicates
  predicateComboBox->addItems(PREDICATES);
  predicateComboBox->setCurrentText(predicate);
  tableWidget->setCellWidget(newRow, 1, predicateComboBox);
}


void SPOEditor::removeSelectedRow()
{
  // Remove the currently selected row
  QList<QTableWidgetSelectionRange> selectedRanges = tableWidget->selectedRanges();
  if (selectedRanges.isEmpty()) {
    QMessageBox::warning(this, "Warning", "No row selected for removal.");
    return;
  }

  // Delete rows starting from the last selected to prevent index issues
  for (int i = selectedRanges.size() - 1; i >= 0; --i) {
    tableWidget->removeRow(selectedRanges.at(i).topRow());
  }
}

}   // namespace rqt_human_radar
