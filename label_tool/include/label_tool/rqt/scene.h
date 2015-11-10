/**
 * \class Scene
 *
 * \brief basic class QGraphicScene
 *
 * Class has same functionalites as QGraphicsScene but signals all mouse events
 * so that they can be handeled somewhere else.
 *
 * \author Julia Nitsch
 *
 * Contact: julia.nitsch@gmail.com
 *
 */
#pragma once

#include <QWidget>

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>

namespace label_tool {


class Scene : public QGraphicsScene
{
  Q_OBJECT

public:
  Scene();
protected:
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *e);
    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *e);
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *e);


signals:
    void mousePressed(QGraphicsSceneMouseEvent* e);
    void mouseRelease(QGraphicsSceneMouseEvent* e);
    void mouseMove(QGraphicsSceneMouseEvent* e);
};
}
