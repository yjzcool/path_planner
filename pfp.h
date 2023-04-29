#pragma once
#include <iostream>
#include <math.h>	//使用pow乘方
#include <vector>	
#include <map>
#include <algorithm>	//vector使用sort算法
#include <float.h> //double数据类型表示无穷大

using namespace std;

//创建一个Point类，用vector容器装点的坐标，再用map容器装 点-力 对
class Point
{
public:
	Point() {};
	Point(double x, double y)
	{
		this->m_x = x;
		this->m_y = y;
	}

	double m_x;
	double m_y;
};

class Pfp
{
public:
	//构造函数
	Pfp();

	//计算吸引力
	double attrPotential(Point& p, Point& pg);

	//计算距离
	double distance(Point& p1, Point& p2);

	//计算斥力
	double repPotential(Point& p, vector<Point>& vObs, double rr);

	//计算合力并且赋给每个点
	void potentialField(Point& ps, Point& pg, vector<Point>& vObs, double reso, double rr);

	//寻找路径
	void findPath(Point& ps, Point& pg, vector<Point>& vObs, double reso, double rr);

public:
	Point pStart; //起点
	Point pGoal;  //终点
	double KP;	//引力增益
	double ETA;	//斥力增益
	double AREA_WIDTH;//地图范围
	double robotRadius; //机器人半径
	double gridSize;//地图分辨率

	vector<Point> vObstract;//装障碍物的容器
	multimap<double, Point> mPoint;//地图的点及对应的势场力
	vector<Point> vMotion;//存放当前点的周围八个方位 
	vector<Point> vPath;//存放路径中的各个点
};

//计算障碍物中的x,y最小值 定义sort的排序方式
class compareX
{
public:
	bool operator()(Point& p1, Point& p2)
	{
		return p1.m_x > p2.m_x;
	}
};

class compareY
{
public:
	bool operator()(Point& p1, Point& p2)
	{
		return p1.m_y > p2.m_y;
	}
};