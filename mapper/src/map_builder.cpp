/* map_builder.cpp
 *
 *  Created on: 03.05.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Department of Computer Science and Control Systems
 */

#include <mapper.h>

Mapper::Map_builder::Map_builder(double x, double y)
{
	add(0,0);
	add(x,y);
}

void Mapper::Map_builder::div_by_two()
{
	max_dist = get_dist(Head, Tail);
	while (max_dist > CELL )
	{
		Point *temp = Head;
		int i =0;
		while (temp->Next != NULL)
		{
			Point *new_point = new Point;
			new_point->Next = temp->Next;
			new_point->x = (temp->x + temp->Next->x)/2;
			new_point->y = (temp->y + temp->Next->y)/2;
			temp->Next = new_point;
			new_point->Prev = temp;
			
			temp = new_point->Next;
			temp->Prev = new_point;
			i++;
		}
		max_dist = get_dist(temp, temp->Prev);	
	}
}

void Mapper::Map_builder::to_map(double yaw, double x_err, double y_err, int8_t map_[CELL_N][CELL_N])
{
	static int8_t i = -127;
	Point *temp = Head;
	while (temp != NULL)
	{
		double dist;

		temp->x = temp->x + x_err;
		temp->y = temp->y + y_err;

		double xR = temp->x*cos(yaw) - temp->y*sin(yaw);
		double yR = temp->y*cos(yaw) + temp->x*sin(yaw);

		if (xR > -0.02 && xR < 0.02) xR = R_POSE;
		else xR = R_POSE+(xR+CELL_H)/CELL;
		if (yR > -0.02 && yR < 0.02) yR = R_POSE;
		else yR = R_POSE+(yR+CELL_H)/CELL;

		if (xR < 0) xR = 0;
		if (yR < 0) yR = 0;

		dist = sqrt(pow(temp->x, 2) + pow(temp->y, 2));
		if (temp != Tail) map_[(int)yR][(int)xR] = 0;
		else if (dist < 0.57) map_[(int)yR][(int)xR] = 100;

		temp = temp->Next;
	}
}

double Mapper::Map_builder::get_dist(Point *P1, Point *P2)
{
	return sqrt(pow(P1->x - P2->x, 2) + pow(P1->y - P2->y, 2));
}

void Mapper::Map_builder::add(double x_, double y_)
{
	Point *temp = new Point;
	temp -> Next = NULL;
	temp -> x = x_;
	temp -> y = y_;
 
	if (Head != NULL)
	{
		temp -> Prev = Tail;
        Tail -> Next = temp;
        Tail = temp;
    }
    else
    {
        temp -> Prev = NULL;
        Head = Tail = temp;
    }
}

Mapper::Map_builder::~Map_builder()
{
	while (Head)
    {
        	Tail=Head->Next;
        	delete Head;
        	Head=Tail;
 	}
}
