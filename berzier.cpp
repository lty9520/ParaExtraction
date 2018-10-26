#include <iostream>  
#include <vector>  
#include <cmath>  
#include <fstream>
#include <cassert>
#include <string>

#define POINTNUM 6

using namespace std;

//最小二乘拟合相关函数定义  
double calcR(vector<double> vy, vector<double> vx, int n, double coefficient[]);
double sum(vector<double> Vnum, int n);
double MutilSum(vector<double> Vx, vector<double> Vy, int n);
double RelatePow(vector<double> Vx, int n, int ex);
double RelateMutiXY(vector<double> Vx, vector<double> Vy, int n, int ex);
void EMatrix(vector<double> Vx, vector<double> Vy, int n, int ex, double coefficient[]);
void CalEquation(int exp, double coefficient[]);
void readTxt(string file);
double F(double c[], int l, int m);
double Em[POINTNUM][4];


//主函数，这里将数据拟合成二次曲线  
int main(int argc, char* argv[])
{
	/*
	double arry1[POINTNUM] = { 776.543, 767.879, 759.214, 750.550, 741.885, 733.221 };
	double arry2[POINTNUM] = { 163.753, 172.569, 180.063, 184.165, 186.899, 186.147 };
	double coefficient[POINTNUM];
	memset(coefficient, 0, sizeof(double) * POINTNUM);
	vector<double> vx, vy;
	for (int i = 0; i < POINTNUM; i++)
	{
		vx.push_back(arry1[i]);
		vy.push_back(arry2[i]);
	}
	EMatrix(vx, vy, POINTNUM, 3, coefficient);
	printf("拟合方程为：y = %lf + %lfx + %lfx^2 \n", coefficient[1], coefficient[2], coefficient[3]);
	double R2;
	R2 = calcR(vy, vx, POINTNUM, coefficient);
	cout << "R ^ 2 = " << R2 << endl;
	*/
	readTxt("read_test.txt");
	system("pause");
	
	return 0;
}
//判定系数
double calcR(vector<double> vy, vector<double> vx, int n, double coefficient[])
{
	double ysum = 0.0;
	for (int i = 0; i < n; i++)
	{
		ysum += vy[i];
	}
	double yaver = ysum / n;
	double sstot = 0.0;
	for (int i = 0; i < n; i++)
	{
		sstot += (vy[i] - yaver) * (vy[i] - yaver);
	}
	vector<double> ey;
	for (int i = 0; i < n; i++)
	{
		ey.push_back((coefficient[3] * vx[i] * vx[i] + coefficient[2] * vx[i] + coefficient[1]));
	}
	double ssres = 0.0;
	for (int i = 0; i < n; i++)
	{
		ssres += (vy[i] - ey[i]) * (vy[i] - ey[i]);
	}
	double R2 = 1 - ssres / sstot;
	return R2;
}
//累加  
double sum(vector<double> Vnum, int n)
{
	double dsum = 0;
	for (int i = 0; i < n; i++)
	{
		dsum += Vnum[i];
	}
	return dsum;
}
//乘积和  
double MutilSum(vector<double> Vx, vector<double> Vy, int n)
{
	double dMultiSum = 0;
	for (int i = 0; i < n; i++)
	{
		dMultiSum += Vx[i] * Vy[i];
	}
	return dMultiSum;
}
//ex次方和  
double RelatePow(vector<double> Vx, int n, int ex)
{
	double ReSum = 0;
	for (int i = 0; i < n; i++)
	{
		ReSum += pow(Vx[i], ex);
	}
	return ReSum;
}
//x的ex次方与y的乘积的累加  
double RelateMutiXY(vector<double> Vx, vector<double> Vy, int n, int ex)
{
	double dReMultiSum = 0;
	for (int i = 0; i < n; i++)
	{
		dReMultiSum += pow(Vx[i], ex)*Vy[i];
	}
	return dReMultiSum;
}
//计算方程组的增广矩阵  
void EMatrix(vector<double> Vx, vector<double> Vy, int n, int ex, double coefficient[])
{
	for (int i = 1; i <= ex; i++)
	{
		for (int j = 1; j <= ex; j++)
		{
			Em[i][j] = RelatePow(Vx, n, i + j - 2);
		}
		Em[i][ex + 1] = RelateMutiXY(Vx, Vy, n, i - 1);
	}
	Em[1][1] = n;
	CalEquation(ex, coefficient);
}
//求解方程  
void CalEquation(int exp, double coefficient[])
{
	for (int k = 1; k < exp; k++) //消元过程  
	{
		for (int i = k + 1; i < exp + 1; i++)
		{
			double p1 = 0;

			if (Em[k][k] != 0)
				p1 = Em[i][k] / Em[k][k];

			for (int j = k; j < exp + 2; j++)
				Em[i][j] = Em[i][j] - Em[k][j] * p1;
		}
	}
	coefficient[exp] = Em[exp][exp + 1] / Em[exp][exp];
	for (int l = exp - 1; l >= 1; l--)   //回代求解  
		coefficient[l] = (Em[l][exp + 1] - F(coefficient, l + 1, exp)) / Em[l][l];
}
//供CalEquation函数调用  
double F(double c[], int l, int m)
{
	double sum = 0;
	for (int i = l; i <= m; i++)
		sum += Em[l - 1][i] * c[i];
	return sum;
}
//读取文件进行拟合方程
void readTxt(string file)
{
	double R2;
	double coefficient[POINTNUM];
	memset(coefficient, 0, sizeof(double) * POINTNUM);
	vector<double> x;
	vector<double> y;
	FILE *fp = NULL;
	char fileName[32] = "read_test.txt";

	fp = fopen(fileName, "r+");

	if (fp == NULL)
	{
		cout << "open failed ";
		cout << "error" <<endl;
	}
	cout << "open success" << endl;
	//读取每行数据
	int num = 1;
	while (!feof(fp))
	{
		char str[1024];
		char *p;
		//char *fgets(char *buf, int bufsize, FILE *stream);
		//*buf: 字符型指针，指向用来存储所得数据的地址。
		//bufsize: 整型数据，指明存储数据的大小。
		// *stream: 文件结构体指针，将要读取的文件流。
		if (fgets(str, sizeof(str), fp) == NULL)
			break;

		//char *strtok(char s[], const char *delim);
		//分解字符串为一组字符串。s为要分解的字符串，delim为分隔符字符串。
		string content = str;
		if (content == "l\n")
			continue;
		if (content == "end\n" || content == "end")
		{
			EMatrix(x, y, POINTNUM, 3, coefficient);
			R2 = calcR(y, x, POINTNUM, coefficient);
			cout << "*********line"<< num << ": " << endl;
			
			cout << "拟合方程为：y =" << coefficient[1] << " + " << coefficient[2] << "x + " << coefficient[3] << "x ^ 2 " << endl;
			cout << "R ^ 2 = " << R2 << endl;
			cout << endl;
			x.clear();
			y.clear();
			num++;
			continue;
		}
		p = strtok(str, " ");
		x.push_back(atof(p));
		while (p)
		{
			//int atoi(const char *nptr);
			//atoi( ) 函数会扫描参数 nptr字符串，
			//跳过前面的空白字符（例如空格，tab缩进等，可以通过isspace( )函数来检测），
			//直到遇上数字或正负符号才开始做转换，而再遇到非数字或字符串结束时('\0')才结束转换，
			//并将结果返回。如果 nptr不能转换成 int 或者 nptr为空字符串，那么将返回 0
			//printf("%d\n", atoi(p));
			//cout << p << endl;
			p = strtok(NULL, " ");
			if (p != NULL)
				y.push_back(atof(p));
			else
				break;			
		}

		
		//cout << endl;
	}
	

	fclose(fp);

	/*
	cout << "test result" << endl;
	for (int i = 0; i < x.size(); i++)
	{
		cout << "x[" << i << "] = " << x[i] << endl;
	}

	for (int i = 0; i < y.size(); i++)
	{
		cout << "y[" << i << "] = " << y[i] << endl;
	}
	*/

	cout<< "finish" << endl;
}
