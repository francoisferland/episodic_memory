/*
 * MyApp.h
 *
 *  Created on: 2012-10-30
 *      Author: frank
 */

#ifndef MYAPP_H_
#define MYAPP_H_

#include <QtGui>
#include <QApplication>

class MyApp : public QApplication {
public:
	MyApp(int& argc, char ** argv) :
    QApplication(argc, argv) { }
  virtual ~MyApp() { }

  // reimplemented from QApplication so we can throw exceptions in slots
  virtual bool notify(QObject * receiver, QEvent * event) {
    try {
      return QApplication::notify(receiver, event);
    } catch(std::exception& e) {
      qCritical() << "Exception thrown:" << e.what();
    }
    return false;
  }
};


#endif /* MYAPP_H_ */
