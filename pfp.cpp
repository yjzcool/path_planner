#include "pfp.h"

Pfp::Pfp()
{
	this->pStart.m_x = 0.0;
	this->pStart.m_y = 10.0;
	this->pGoal.m_x = 30.0;
	this->pGoal.m_y = 30.0;
	this->AREA_WIDTH = 30.0;
	this->robotRadius = 0.5;
	this->gridSize = 0.5;
	this->KP = 5.0;
	this->ETA = 100.0;

	Point p1(15.0, 25.0);
	Point p2(5.0, 15.0);
	Point p3(20.0, 26.0);
	Point p4(25.0, 25.0);
	
	this->vObstract.push_back(p1);
	this->vObstract.push_back(p2);
	this->vObstract.push_back(p3);
	this->vObstract.push_back(p4);

	Point right(1.0, 0.0);
	Point up(0.0, 1.0);
	Point left(-1.0, 0.0);
	Point down(0.0, -1.0);
	Point leftDown(-1.0, -1.0);
	Point leftUp(-1.0, 1.0);
	Point rightDown(1.0, -1.0);
	Point rightUp(1.0, 1.0);

	this->vMotion.push_back(right);
	this->vMotion.push_back(up);
	this->vMotion.push_back(left);
	this->vMotion.push_back(down);	
	this->vMotion.push_back(leftDown);
	this->vMotion.push_back(leftUp);
	this->vMotion.push_back(rightDown);
	this->vMotion.push_back(rightUp);
}

double Pfp::attrPotential(Point& p, Point& pg)
{
	return 0.5 * this->KP * distance(p, pg);
}

double Pfp::distance(Point& p1, Point& p2)
{
	double dis = pow((p1.m_x - p2.m_x), 2) + pow((p1.m_y - p2.m_y), 2);
	dis = pow(dis, 0.5);
	return dis;
}

double Pfp::repPotential(Point& p, vector<Point>& vObs, double rr)
{
	double disMin = distance(p, vObs[0]);
	//寻找最近障碍物
	for (int i = 0; i < vObs.size(); i++)
	{
		if (disMin > distance(p, vObs[i]))
		{
			disMin = distance(p, vObs[i]);
		}
	}

	if (disMin <= rr)
	{
		if (disMin <= 0.1)//相当于碰撞到障碍物了
		{
			disMin = 0.1;
		}
		return 0.5 * this->ETA * pow((1.0 / disMin - 1.0 / rr), 2);
	}
	else
	{
		return 0.0;
	}
}

//获得每个像素点的势场合力
void Pfp::potentialField(Point& ps, Point& pg, vector<Point>& vObs, 
	double reso, double rr)
{
	sort(vObs.begin(), vObs.end(), compareX());//按照x的递减排列
	double minx = vObs.back().m_x - AREA_WIDTH / 2.0;//-10
	double maxx = vObs.front().m_x + AREA_WIDTH / 2.0;//40

	sort(vObs.begin(), vObs.end(), compareY());//按照y的递减排列
	double miny = vObs.back().m_y - AREA_WIDTH / 2.0;//0
	double maxy = vObs.front().m_y + AREA_WIDTH / 2.0;//41

	//计算x，y方向所有的像素点
	int xw = int((maxx - minx) / reso);
	int yw = int((maxy - miny) / reso);

	//cout << "xw" << xw << " " << "yw" << yw << endl;//100 82

	
	double uf = 0.0;
	double ua = 0.0;
	double ur = 0.0;

	for (int i = 0; i < xw; i++)
	{
		for (int j = 0; j < yw; j++)
		{
			Point p;
			p.m_x = i * reso + minx;
			p.m_y = j * reso + miny;
			//cout << "x: " << p.m_x << " y: " << p.m_y;

			ua = this->attrPotential(p, this->pGoal);
			ur = this->repPotential(p, this->vObstract, this->robotRadius);
			uf = ua + ur;
			//cout << " ua: " << ua << "ur: "<< ur << "uf: "<< uf << endl;
			this->mPoint.insert(make_pair(uf, p));
		}
	}
	
	//cout << "map大小" << mPoint.size() << endl; //8200

	//值找键 测试
	//Point pt(39, 0.5);
	//for (multimap<double, Point>::iterator it = mPoint.begin(); it != mPoint.end(); it++)
	//{
	//	//cout << "x: " << it->second.m_x << "\ty: " << it->second.m_y << "\tuf: " << it->first << endl;
	//	if (it->second.m_x == pt.m_x && it->second.m_y == pt.m_y)
	//	{
	//		cout << "对应的键为：" << it->first << endl;
	//		break;
	//	}
	//}
}


//关键在遍历当前点周围8个方向的势场力选最小
void Pfp::findPath(Point& ps, Point& pg, vector<Point>& vObs, double reso, double rr)
{
	this->vPath.push_back(ps);

	Point pCurrent(ps);	//起始时在起点

	double ix;//邻近点坐标
	double iy;
	double iUf;//邻近点势场力
	
	double minx;
	double miny;
	double minUf = DBL_MAX;

	//只要不到终点（dis < rr）就一直循环
	while (distance(pCurrent, this->pGoal) > reso)
	{
		for (int i = 0; i < this->vMotion.size(); i++)
		{
			ix = pCurrent.m_x + this->vMotion[i].m_x;
			iy = pCurrent.m_y + this->vMotion[i].m_y;

			//值找键
			for (multimap<double, Point>::iterator it = this->mPoint.begin(); it != this->mPoint.end(); it++)
			{
				if (it->second.m_x == ix && it->second.m_y == iy)
				{
					iUf = it->first;
					break;
				}
			}

			//找到势场力最小的邻近点
			if (iUf < minUf)
			{
				minx = ix;
				miny = iy;
				minUf = iUf;
			}
		}
		pCurrent.m_x = minx;
		pCurrent.m_y = miny;
		this->vPath.push_back(pCurrent);
	}
	
	//cout << "Get goal!" << endl;
	////遍历路径
	//for (vector<Point>::iterator it = this->vPath.begin(); it != this->vPath.end(); it++)
	//{
	//	cout << "x:" << it->m_x << "\ty:" << it->m_y << endl;
	//}
}
