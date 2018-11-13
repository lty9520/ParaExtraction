#include "berzier.h"

berzier_fit::berzier_fit()
{

}

berzier_fit::~berzier_fit()
{

}




//�ж�ϵ��
//************************************
// Method:    calcR
// FullName:  berzier_fit::calcR
// Access:    public 
// Returns:   double
// Qualifier:
// Parameter: vector<double> vy
// Parameter: vector<double> vx
// Parameter: int n
// Parameter: double coefficient[]
//************************************
double berzier_fit::calcR(vector<double> vy, vector<double> vx, int n, double coefficient[])
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
//�ۼ�  
//************************************
// Method:    sum
// FullName:  berzier_fit::sum
// Access:    public 
// Returns:   double
// Qualifier:
// Parameter: vector<double> Vnum
// Parameter: int n
//************************************
double berzier_fit::sum(vector<double> Vnum, int n)
{
	double dsum = 0;
	for (int i = 0; i < n; i++)
	{
		dsum += Vnum[i];
	}
	return dsum;
}
//�˻���  
//************************************
// Method:    MutilSum
// FullName:  berzier_fit::MutilSum
// Access:    public 
// Returns:   double
// Qualifier:
// Parameter: vector<double> Vx
// Parameter: vector<double> Vy
// Parameter: int n
//************************************
double berzier_fit::MutilSum(vector<double> Vx, vector<double> Vy, int n)
{
	double dMultiSum = 0;
	for (int i = 0; i < n; i++)
	{
		dMultiSum += Vx[i] * Vy[i];
	}
	return dMultiSum;
}
//ex�η���  
//************************************
// Method:    RelatePow
// FullName:  berzier_fit::RelatePow
// Access:    public 
// Returns:   double
// Qualifier:
// Parameter: vector<double> Vx
// Parameter: int n
// Parameter: int ex
//************************************
double berzier_fit::RelatePow(vector<double> Vx, int n, int ex)
{
	double ReSum = 0;
	for (int i = 0; i < n; i++)
	{
		ReSum += pow(Vx[i], ex);
	}
	return ReSum;
}
//x��ex�η���y�ĳ˻����ۼ�  
//************************************
// Method:    RelateMutiXY
// FullName:  berzier_fit::RelateMutiXY
// Access:    public 
// Returns:   double
// Qualifier:
// Parameter: vector<double> Vx
// Parameter: vector<double> Vy
// Parameter: int n
// Parameter: int ex
//************************************
double berzier_fit::RelateMutiXY(vector<double> Vx, vector<double> Vy, int n, int ex)
{
	double dReMultiSum = 0;
	for (int i = 0; i < n; i++)
	{
		dReMultiSum += pow(Vx[i], ex)*Vy[i];
	}
	return dReMultiSum;
}
//���㷽������������  
//************************************
// Method:    EMatrix
// FullName:  berzier_fit::EMatrix
// Access:    public 
// Returns:   void
// Qualifier:
// Parameter: vector<double> Vx
// Parameter: vector<double> Vy
// Parameter: int n
// Parameter: int ex
// Parameter: double coefficient[]
//************************************
void berzier_fit::EMatrix(vector<double> Vx, vector<double> Vy, int n, int ex, double coefficient[])
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
//��ⷽ��  
//************************************
// Method:    CalEquation
// FullName:  berzier_fit::CalEquation
// Access:    public 
// Returns:   void
// Qualifier:
// Parameter: int exp
// Parameter: double coefficient[]
//************************************
void berzier_fit::CalEquation(int exp, double coefficient[])
{
	for (int k = 1; k < exp; k++) //��Ԫ����  
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
	for (int l = exp - 1; l >= 1; l--)   //�ش����  
		coefficient[l] = (Em[l][exp + 1] - F(coefficient, l + 1, exp)) / Em[l][l];
}
//��CalEquation��������  
//************************************
// Method:    F
// FullName:  berzier_fit::F
// Access:    public 
// Returns:   double
// Qualifier:
// Parameter: double c[]
// Parameter: int l
// Parameter: int m
//************************************
double berzier_fit::F(double c[], int l, int m)
{
	double sum = 0;
	for (int i = l; i <= m; i++)
		sum += Em[l - 1][i] * c[i];
	return sum;
}
//��ȡ�ļ�������Ϸ���
//************************************
// Method:    readTxt
// FullName:  berzier_fit::readTxt
// Access:    public 
// Returns:   void
// Qualifier:
// Parameter: string file
//************************************
void berzier_fit::readTxt(string file)
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
		cout << "error" << endl;
	}
	cout << "open success" << endl;
	//��ȡÿ������
	int num = 1;
	while (!feof(fp))
	{
		char str[1024];
		char *p;
		//char *fgets(char *buf, int bufsize, FILE *stream);
		//*buf: �ַ���ָ�룬ָ�������洢�������ݵĵ�ַ��
		//bufsize: �������ݣ�ָ���洢���ݵĴ�С��
		// *stream: �ļ��ṹ��ָ�룬��Ҫ��ȡ���ļ�����
		if (fgets(str, sizeof(str), fp) == NULL)
			break;

		//char *strtok(char s[], const char *delim);
		//�ֽ��ַ���Ϊһ���ַ�����sΪҪ�ֽ���ַ�����delimΪ�ָ����ַ�����
		string content = str;
		if (content == "l\n")
			continue;
		if (content == "end\n" || content == "end")
		{
			EMatrix(x, y, POINTNUM, 3, coefficient);
			R2 = calcR(y, x, POINTNUM, coefficient);
			cout << "*********line" << num << ": " << endl;

			cout << "��Ϸ���Ϊ��y =" << coefficient[1] << " + " << coefficient[2] << "x + " << coefficient[3] << "x ^ 2 " << endl;
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
			//atoi( ) ������ɨ����� nptr�ַ�����
			//����ǰ��Ŀհ��ַ�������ո�tab�����ȣ�����ͨ��isspace( )��������⣩��
			//ֱ���������ֻ��������Ųſ�ʼ��ת�����������������ֻ��ַ�������ʱ('\0')�Ž���ת����
			//����������ء���� nptr����ת���� int ���� nptrΪ���ַ�������ô������ 0
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

	cout << "finish" << endl;
}