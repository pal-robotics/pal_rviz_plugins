#ifndef MAP_MANAGEMENT_PANEL
#define MAP_MANAGEMENT_PANEL

#include <rviz/panel.h>

namespace pal
{


class MapManagementPanel: public rviz::Panel
{
  Q_OBJECT
public:
  MapManagementPanel( QWidget* parent = 0 );

  ~MapManagementPanel();

  virtual void onInitialize() override;
};
}

#endif // MAP_MANAGEMENT_PANEL

