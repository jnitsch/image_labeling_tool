#include <label_tool/rqt/scene.h>

namespace label_tool {
Scene::Scene() :
  QGraphicsScene()
{

}

void Scene::mousePressEvent(QGraphicsSceneMouseEvent *e)
{
  // forward signal to gui handler -> signals are destroyed by qt after sending,
  // so need to copy received signal to new signal
  QGraphicsSceneMouseEvent *new_event = new QGraphicsSceneMouseEvent();
  new_event->setAccepted(e->isAccepted());
  new_event->setButton(e->button());
  new_event->setButtons(e->buttons());
  new_event->setLastPos(e->lastPos());
  new_event->setLastScenePos(e->lastScenePos());
  new_event->setLastScreenPos(e->lastScreenPos());
  new_event->setModifiers(e->modifiers());
  new_event->setPos(e->pos());
  new_event->setScenePos(e->scenePos());
  new_event->setScreenPos(e->screenPos());
  emit mousePressed(new_event);
}

void Scene::mouseReleaseEvent(QGraphicsSceneMouseEvent *e)
{
  QGraphicsSceneMouseEvent *new_event = new QGraphicsSceneMouseEvent();
  new_event->setAccepted(e->isAccepted());
  new_event->setButton(e->button());
  new_event->setButtons(e->buttons());
  new_event->setLastPos(e->lastPos());
  new_event->setLastScenePos(e->lastScenePos());
  new_event->setLastScreenPos(e->lastScreenPos());
  new_event->setModifiers(e->modifiers());
  new_event->setPos(e->pos());
  new_event->setScenePos(e->scenePos());
  new_event->setScreenPos(e->screenPos());
  emit mouseRelease(new_event);
}

void Scene::mouseMoveEvent(QGraphicsSceneMouseEvent *e)
{
  QGraphicsSceneMouseEvent *new_event = new QGraphicsSceneMouseEvent();
  new_event->setAccepted(e->isAccepted());
  new_event->setButton(e->button());
  new_event->setButtons(e->buttons());
  new_event->setLastPos(e->lastPos());
  new_event->setLastScenePos(e->lastScenePos());
  new_event->setLastScreenPos(e->lastScreenPos());
  new_event->setModifiers(e->modifiers());
  new_event->setPos(e->pos());
  new_event->setScenePos(e->scenePos());
  new_event->setScreenPos(e->screenPos());
  emit mouseMove(new_event);
}


}
