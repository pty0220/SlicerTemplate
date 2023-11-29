/*==============================================================================

  Copyright (c) Kitware, Inc.

  See http://www.slicer.org/copyright/copyright.txt for details.

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

  This file was originally developed by Julien Finet, Kitware, Inc.
  and was partially funded by NIH grant 3P41RR013218-12S1

==============================================================================*/

#ifndef __qsnmAppMainWindow_h
#define __qsnmAppMainWindow_h

// snm includes
#include "qsnmAppExport.h"
class qsnmAppMainWindowPrivate;

// Slicer includes
#include "qSlicerMainWindow.h"

class Q_SNM_APP_EXPORT qsnmAppMainWindow : public qSlicerMainWindow
{
  Q_OBJECT
public:
  typedef qSlicerMainWindow Superclass;

  qsnmAppMainWindow(QWidget *parent=0);
  virtual ~qsnmAppMainWindow();

public slots:
  void on_HelpAboutsnmAppAction_triggered();

protected:
  qsnmAppMainWindow(qsnmAppMainWindowPrivate* pimpl, QWidget* parent);

private:
  Q_DECLARE_PRIVATE(qsnmAppMainWindow);
  Q_DISABLE_COPY(qsnmAppMainWindow);
};

#endif
