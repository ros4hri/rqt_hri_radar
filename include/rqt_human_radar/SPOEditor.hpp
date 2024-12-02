#pragma once

#include <QDialog>
#include <QTableWidget>
#include <tuple>
#include <string>

namespace rqt_human_radar
{

typedef std::tuple<std::string, std::string, std::string> Triple;

class SPOEditor : public QDialog
{
  Q_OBJECT

public:
  SPOEditor(const std::string & subject = "", QWidget * parent = nullptr);

  void setTriples(const std::vector<Triple> & spo);

  std::vector<Triple> getTriples() const;

private slots:
  void addRow();
  void removeSelectedRow();

private:
  QTableWidget * tableWidget;

  std::string subject_;
};

}   // namespace rqt_human_radar
