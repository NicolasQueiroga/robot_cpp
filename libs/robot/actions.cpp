#include "robot/actions.hpp"

Actions::Actions(ros::NodeHandle *nh) : Robot(nh)
{
    // Infos
    p = {0.0, 0.0};
    rad = 0.0;
    dist = 0.0;

    // makeSquare
    turn = false;
    walk = true;
    angles[0] = 90;
    angles[1] = 180;
    angles[2] = 270;
    i = 0;
    p1 = {0.0, 0.0};

    // followRoad
    limit = 10;
}

void Actions::makeSquare()
{
    getDistance(&dist);
    getPosition(&p, &rad);
    double d = pow((pow((p.x - p1.x), 2) + pow((p.y - p1.y), 2)), 0.5);

    if (walk && !turn)
    {
        std::cout << "debug: walk\n";
        if (d > 0.8)
        {
            p1 = p;
            setSpeed();
            walk = false;
            if (i == 3)
                turn = false;
            else
                turn = true;
        }
        else
            setSpeed(0.3, 0);
    }
    else if (!walk && turn)
    {
        if (rad >= angles[i] && rad < angles[i] + 5)
        {
            std::cout << "\n\n\ndebug: turn ENDED\n\n\n";
            setSpeed();
            turn = false;
            walk = true;
            i++;
        }
        else
        {
            std::cout << "debug: turn\n";
            setSpeed(0, 0.3);
        }
    }
    else
    {
        if (rad >= 0 && rad < 3)
        {
            i = 0;
            walk = true;
        }
        else
            setSpeed(0, 0.3);
    }
}

void Actions::followRoad()
{
    if ((pcenter.x - limit <= pmin.x) && ((pcenter.x + limit >= pmin.x)))
    {
        setSpeed(0.3, 0);
        if ((angularCoef > -0.19) && (angularCoef < 0.19))
            setSpeed(0.42, 0);
    }
    else
    {
        double delta = pmin.x - pcenter.x;
        double max_delta = pcenter.x;
        double k = (delta / max_delta) * 0.25 + tan(angularCoef) / 360;
        if (k != k)
            setSpeed(0, 0.1);
        else
            setSpeed(0.2, -k);
    }
}