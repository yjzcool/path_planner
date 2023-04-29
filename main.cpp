#include "pfp.h"
#include <graphics.h>

int main()
{
	Pfp pfp;
	pfp.potentialField(pfp.pStart, pfp.pGoal, pfp.vObstract, pfp.gridSize, pfp.robotRadius);
	pfp.findPath(pfp.pStart, pfp.pGoal, pfp.vObstract, pfp.gridSize, pfp.robotRadius);

	//��ͼ
	initgraph(1100, 900);//��10������Ϊһ��դ��
	setbkcolor(WHITE);//���ñ���ɫΪ��ɫ
	cleardevice();//�Ա���ɫ�����Ļ
	
	setlinecolor(BLACK);//���û�ͼɫΪ��
	for (int i = 0; i < 100; i++)
	{
		for (int j = 0; j < 82; j++)
		{
			rectangle(i * 10, j * 10, i * 10 + 10, j * 10 + 10);
		}
	}

	setfillcolor(RED);//�����ϰ������ɫΪ��ɫ
	for (int i = 0; i < pfp.vObstract.size(); i++)
	{
		fillrectangle(pfp.vObstract[i].m_x * 10, pfp.vObstract[i].m_y * 10, pfp.vObstract[i].m_x * 10 + 10, pfp.vObstract[i].m_y * 10 + 10);
	}

	setfillcolor(GREEN);//����·��Ϊ��ɫ
	for (int i = 0; i < pfp.vPath.size(); i++)
	{
		fillrectangle(pfp.vPath[i].m_x * 10, pfp.vPath[i].m_y * 10, pfp.vPath[i].m_x * 10 + 10, pfp.vPath[i].m_y * 10 + 10);
	}

	setfillcolor(YELLOW);//��������յ�Ϊ��ɫ
	fillrectangle(pfp.pStart.m_x * 10, pfp.pStart.m_y * 10, pfp.pStart.m_x * 10 + 10, pfp.pStart.m_y * 10 + 10);
	fillrectangle(pfp.pGoal.m_x * 10, pfp.pGoal.m_y * 10, pfp.pGoal.m_x * 10 + 10, pfp.pGoal.m_y * 10 + 10);

	system("pause");
	return 0;
}