#include "lineFit.h"

lineFit::lineFit()
{

}

lineFit::~lineFit()
{

}

//************************************
// Method:    LineFitLeastSquares
// FullName:  lineFit::LineFitLeastSquares
// Access:    public 
// Returns:   void
// Qualifier:
// Parameter: vector<double> data_x
// Parameter: vector<double> data_y
// Parameter: int data_n
// Parameter: double * vResult
//************************************
void lineFit::LineFitLeastSquares(vector<double> data_x, vector<double> data_y, int data_n, double *vResult)
{
	double A = 0.0;
	double B = 0.0;
	double C = 0.0;
	double D = 0.0;
	double E = 0.0;
	double F = 0.0;

	for (int i = 0; i<data_n; i++)
	{
		A += data_x[i] * data_x[i];
		B += data_x[i];
		C += data_x[i] * data_y[i];
		D += data_y[i];
	}

	// 计算斜率a和截距b  
	double a, b, temp = 0;
	if (temp = (data_n*A - B*B))// 判断分母不为0  
	{
		a = (data_n*C - B*D) / temp;
		b = (A*D - B*C) / temp;
	}
	else
	{
		a = 1;
		b = 0;
	}

	// 计算相关系数r  
	double Xmean, Ymean;
	Xmean = B / data_n;
	Ymean = D / data_n;

	double tempSumXX = 0.0, tempSumYY = 0.0;
	for (int i = 0; i<data_n; i++)
	{
		tempSumXX += (data_x[i] - Xmean) * (data_x[i] - Xmean);
		tempSumYY += (data_y[i] - Ymean) * (data_y[i] - Ymean);
		E += (data_x[i] - Xmean) * (data_y[i] - Ymean);
	}
	F = sqrt(tempSumXX) * sqrt(tempSumYY);

	double r;
	r = E / F;

	vResult[0] = a;
	vResult[1] = b;
	vResult[2] = r *r;
}


//************************************
// Method:    GetCrossPoint
// FullName:  lineFit::GetCrossPoint
// Access:    public 
// Returns:   void
// Qualifier:
// Parameter: double a1
// Parameter: double b1
// Parameter: double c1
// Parameter: double a2
// Parameter: double b2
// Parameter: double c2
// Parameter: double * cross_Pt
//************************************
void lineFit::GetCrossPoint(double a1, double b1, double c1, double a2, double b2, double c2, double *cross_Pt)
{

	double D = a1*b2 - a2*b1;
	double x = (b1*c2 - b2*c1) / D;
	double y = (a2*c1 - a1*c2) / D;
	cross_Pt[0] = x;
	cross_Pt[1] = y;
}


//************************************
// Method:    GetPointData
// FullName:  lineFit::GetPointData
// Access:    public 
// Returns:   void
// Qualifier:
// Parameter: string filename
// Parameter: vector<double> & py
// Parameter: vector<double> & pz
//************************************
void lineFit::GetPointData(string filename, vector<double> &py, vector<double> &pz){
	ifstream file_read(filename);

	string line = "";
	vector<string> temp;

	if (file_read) // 有该文件  
	{
		int j = 0;
		while (getline(file_read, line)) // line中不包括每行的换行符  
		{

			boost::split(temp, line, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp.size() - 1; i++)
			{
				py[j] = stod(temp[0]);
				pz[j] = stod(temp[1]);
				j++;
			}

		}
		/*
		for (int i = 0; i < py_line1.size(); i++){

		cout << py_line1[i];
		cout << "\t";
		cout << pz_line1[i];
		cout << "\n";
		}
		*/
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}
}
