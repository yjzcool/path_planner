#include "pfp.h"
#include <graphics.h>

int main()
{
	Pfp pfp;
	pfp.potentialField(pfp.pStart, pfp.pGoal, pfp.vObstract, pfp.gridSize, pfp.robotRadius);
	pfp.findPath(pfp.pStart, pfp.pGoal, pfp.vObstract, pfp.gridSize, pfp.robotRadius);

	//画图
	initgraph(1100, 900);//以10像素作为一个栅格
	setbkcolor(WHITE);//设置背景色为白色
	cleardevice();//以背景色清空屏幕
	
	setlinecolor(BLACK);//设置绘图色为黑
	for (int i = 0; i < 100; i++)
	{
		for (int j = 0; j < 82; j++)
		{
			rectangle(i * 10, j * 10, i * 10 + 10, j * 10 + 10);
		}
	}

	setfillcolor(RED);//设置障碍物填充色为红色
	for (int i = 0; i < pfp.vObstract.size(); i++)
	{
		fillrectangle(pfp.vObstract[i].m_x * 10, pfp.vObstract[i].m_y * 10, pfp.vObstract[i].m_x * 10 + 10, pfp.vObstract[i].m_y * 10 + 10);
	}

	setfillcolor(GREEN);//设置路径为绿色
	for (int i = 0; i < pfp.vPath.size(); i++)
	{
		fillrectangle(pfp.vPath[i].m_x * 10, pfp.vPath[i].m_y * 10, pfp.vPath[i].m_x * 10 + 10, pfp.vPath[i].m_y * 10 + 10);
	}

	setfillcolor(YELLOW);//设置起点终点为黄色
	fillrectangle(pfp.pStart.m_x * 10, pfp.pStart.m_y * 10, pfp.pStart.m_x * 10 + 10, pfp.pStart.m_y * 10 + 10);
	fillrectangle(pfp.pGoal.m_x * 10, pfp.pGoal.m_y * 10, pfp.pGoal.m_x * 10 + 10, pfp.pGoal.m_y * 10 + 10);

	system("pause");
	return 0;
}