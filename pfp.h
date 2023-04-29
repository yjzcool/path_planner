#pragma once
#include <iostream>
#include <math.h>	//ʹ��pow�˷�
#include <vector>	
#include <map>
#include <algorithm>	//vectorʹ��sort�㷨
#include <float.h> //double�������ͱ�ʾ�����

using namespace std;

//����һ��Point�࣬��vector����װ������꣬����map����װ ��-�� ��
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
	//���캯��
	Pfp();

	//����������
	double attrPotential(Point& p, Point& pg);

	//�������
	double distance(Point& p1, Point& p2);

	//�������
	double repPotential(Point& p, vector<Point>& vObs, double rr);

	//����������Ҹ���ÿ����
	void potentialField(Point& ps, Point& pg, vector<Point>& vObs, double reso, double rr);

	//Ѱ��·��
	void findPath(Point& ps, Point& pg, vector<Point>& vObs, double reso, double rr);

public:
	Point pStart; //���
	Point pGoal;  //�յ�
	double KP;	//��������
	double ETA;	//��������
	double AREA_WIDTH;//��ͼ��Χ
	double robotRadius; //�����˰뾶
	double gridSize;//��ͼ�ֱ���

	vector<Point> vObstract;//װ�ϰ��������
	multimap<double, Point> mPoint;//��ͼ�ĵ㼰��Ӧ���Ƴ���
	vector<Point> vMotion;//��ŵ�ǰ�����Χ�˸���λ 
	vector<Point> vPath;//���·���еĸ�����
};

//�����ϰ����е�x,y��Сֵ ����sort������ʽ
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