#include <iostream>
#include <time.h>
#include <windows.h>
/*
	ģ��̶���Ⱦ֡�ʵ���ѭ������
*/
// ���10~20�������
int getRand()
{
	return rand() % (10 - 20 + 1) + 10;
}

double UserRender(int current)
{
	// ִ������Ⱦ���ߣ����ܺķѵ�ʱ���ǲ�ȷ���Եġ�200ms,276ms,1s...
	time_t strat, end;
	time(&strat);
	std::cout << current << std::endl;
	int a = getRand();
	Sleep(a);
	time(&end);
	return difftime(end, strat) * 1000;
}

int main()
{
	time_t start, end;
	// ��ѭ����ڣ��̶�֡���趨Ϊ60fps
	static int excuteCount = 1;
	static int FPS = 60;
	double timeSUM = 0.0;
	double initNum = 1000.0 / FPS;
	double sleepTime =0.;
	for (int i = 0; i < 5; i++)
	{
		time(&start);
		while (excuteCount <= FPS && timeSUM <= 1000.0)
		{

			double curUseTime = UserRender(excuteCount);
			sleepTime = initNum - curUseTime;
			if (sleepTime > 0.)
			{
				Sleep(sleepTime);
				timeSUM += initNum;
			}
			else
			{
				timeSUM += curUseTime;
			}

			excuteCount++;
		}
		time(&end);
		excuteCount = 1;
		timeSUM = 0.;
		std::cout <<"ִ�����ָ��������ʱ��" << difftime(end, start)*1000<< "\n" << std::endl;
	}
	system("pause");
	return 0;
}


