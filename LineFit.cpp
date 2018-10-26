#include <math.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>  
#include <string>
#include <boost/algorithm/string/classification.hpp>  
#include <boost/algorithm/string/split.hpp>
/*************************************************************************
最小二乘法拟合直线，y = a*x + b; n组数据; r-相关系数[-1,1],fabs(r)->1,说明x,y之间线性关系好，fabs(r)->0，x,y之间无线性关系，拟合无意义
a = (n*C - B*D) / (n*A - B*B)
b = (A*D - B*C) / (n*A - B*B)
r = E / F
其中：
A = sum(Xi * Xi)
B = sum(Xi)
C = sum(Xi * Yi)
D = sum(Yi)
E = sum((Xi - Xmean)*(Yi - Ymean))
F = sqrt(sum((Xi - Xmean)*(Xi - Xmean))) * sqrt(sum((Yi - Ymean)*(Yi - Ymean)))


一般方程法：
直线的一般方程为F(x) = ax + by + c = 0。既然我们已经知道直线的两个点，假设为(x0,y0), (x1, y1)，那么可以得到a = y0 – y1, b = x1 – x0, c = x0y1 – x1y0。

因此我们可以将两条直线分别表示为

F0(x) = a0*x + b0*y + c0 = 0, F1(x) = a1*x + b1*y + c1 = 0

那么两条直线的交点应该满足

a0*x + b0*y +c0 = a1*x + b1*y + c1

由此可推出

x = (b0*c1 – b1*c0)/D

y = (a1*c0 – a0*c1)/D

D = a0*b1 – a1*b0， (D为0时，表示两直线重合)

二者实际上就是连立方程组F0(x) = a0*x + b0*y + c0 = 0, F1(x) = a1*x + b1*y + c1 = 0的叉积应用

i     j     k

a0 b0 c0

a1 b1 c1

**************************************************************************/
using namespace std;

void LineFitLeastSquares(vector<double> data_x, vector<double> data_y, int data_n, double *vResult)
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

void GetCrossPoint(double a1, double b1, double c1, double a2, double b2, double c2, double *cross_Pt)
{
	
	double D = a1*b2 - a2*b1;
	double x = (b1*c2 - b2*c1) / D;
	double y = (a2*c1 - a1*c2) / D;
	cross_Pt[0] = x;
	cross_Pt[1] = y;
}


void GetPointData(string filename, vector<double> &py, vector<double> &pz){
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


void main(){
	double result_line1[3] = { 0.0 };
	//line1
	//float py_line1[5] = { 0.542008, 0.546456, 0.550007, 0.555676, 0.561717 };
	//float pz_line1[5] = { 0.167172, 0.167409, 0.16764, 0.167184, 0.167541 };
	//LineFitLeastSquares(py_line1, pz_line1, 5, result_line1);
	//line1:	y = 0.00938124 * x + 0.162176 

	double result_line2[3] = { 0.0 };
	//line2
	//float py_line2[5] = { 0.561717, 0.562809, 0.562933, 0.564044, 0.564226 };
	//float pz_line2[5] = { 0.167172, 0.153758, 0.147251, 0.139211, 0.133453 };
	//LineFitLeastSquares(py_line2, pz_line2, 5, result_line2);
	//line1:	y = -12.8721 * x + 7.40116 

	//line1 X line2
	double cross_l1l2[2] = { 0.0 };
	GetCrossPoint(-0.00938124, 1, -0.162176, 12.8721, 1, -7.40116, cross_l1l2);
	//cross_l1l2[0] = ( result_line2[1] - result_line1[1] ) / ( result_line2[0] - result_line1[0] );
	//cross_l1l2[1] = ( result_line1[1] * cross_l1l2[0] + result_line1[0] );
	//-12.772115789329371		7.3408061638754303
	

	double result_line3[3] = { 0.0 };
	//0.0086426577107487409		0.099254906664914988

	double result_line4[3] = { 0.0 };
	//-24.468813267823940		16.787610662980402

	double result_line5[3] = { 0.0 };
	//0.046652456362890296		0.13736073860141859

	double result_line6[3] = { 0.0 };
	//-0.043495199778486572		0.23412168243631806

	double result_line7[3] = { 0.0 };
	//53.769230940892143		-32.118431026324217

	double result_line8[3] = { 0.0 };
	//-25.981333327253719		16.850769545375389

	double result_line9[3] = { 0.0 };
	//-0.012689657656289533		0.21807928169814617

	double result_line10[3] = { 0.0 };
	//-0.011476867026552189		0.11058555168453009

	double result_line11[3] = { 0.0 };
	//-46.455589052965166		29.975526382861421

	double result_line12[3] = { 0.0 };
	//-48.636533483638686		29.505414965362423

	double result_line13[3] = { 0.0 };
	//-31.345674110139655		17.815280924797410

	double result_line14[3] = { 0.0 };
	//-33.951145236631433		23.220632923314835

	double result_line15[3] = { 0.0 };
	//0.015634672133861809		0.15863562696910430

	double cross_l4l5[2] = { 0.0 };
	GetCrossPoint(24.4688, 1, -16.7876, -0.0466, 1, -0.1374, cross_l4l5);
	//y 0.67917330683022414		z 0.16904584166115555

	double cross_l6l7[2] = { 0.0 };
	GetCrossPoint(0.0434, 1, -0.2341, -53.7692, 1, 32.1184, cross_l6l7);
	//y 0.60120635078379903		z 0.20797209210088186

	double cross_l8l9[2] = { 0.0 };
	GetCrossPoint(25.9813, 1, -16.8508, 0.0126, 1, -0.2180, cross_l8l9);
	//y 0.64049129693861617		z 0.20995166640816224

	double cross_l6l13[2] = { 0.0 };
	GetCrossPoint(0.04350, 1, -0.2341, 31.3456, 1, -17.8152, cross_l6l13);
	//y 0.56165927914179947		z 0.20969219988260474

	double cross_l9l14[2] = { 0.0 };
	GetCrossPoint(0.0126, 1, -0.2180, 33.9511, 1, -23.2206, cross_l9l14);
	//y 0.67777255179127127		z 0.20947858004708517

	double cross_l12l15[2] = { 0.0 };
	GetCrossPoint(48.6365, 1, -29.5054, -0.0156, 1, -0.1586, cross_l12l15);
	//y 0.60319571461711596		z 0.16806639419969338

	double cross_l11l15[2] = { 0.0 };
	GetCrossPoint(46.4556, 1, -29.9755, -0.0156, 1, -0.1586, cross_l11l15);
	//y 0.64162052052415108		z 0.16866715344185715

	
	//line1		point y
	std::vector<double> py_line1_x(5);
	//line1		point z			_x
	std::vector<double> pz_line1_x(5);
	//line2		point y			_x
	std::vector<double> py_line2_x(5);
	//line2		point z			_x
	std::vector<double> pz_line2_x(5);
	//line3		point y			_x
	std::vector<double> py_line3_x(13);
	//line3		point z			_x
	std::vector<double> pz_line3_x(13);
	//line4		point y			_x
	std::vector<double> py_line4_x(4);
	//line4		point z			_x
	std::vector<double> pz_line4_x(4);
	//line5		point y			_x
	std::vector<double> py_line5_x(6);
	//line5		point z			_x
	std::vector<double> pz_line5_x(6);
	//line6		point y			_x
	std::vector<double> py_line6_x(7);
	//line6		point z			_x
	std::vector<double> pz_line6_x(7);
	//line7		point y			_x
	std::vector<double> py_line7_x(2);
	//line7		point z			_x
	std::vector<double> pz_line7_x(2);
	//line8		point y			_x
	std::vector<double> py_line8_x(2);
	//line8		point z			_x
	std::vector<double> pz_line8_x(2);
	//line9		point y			_x
	std::vector<double> py_line9_x(5);
	//line9		point z			_x
	std::vector<double> pz_line9_x(5);
	//line10		point y
	std::vector<double> py_line10_x(13);
	//line10		point z		 _x
	std::vector<double> pz_line10_x(13);
	//line11		point y		 _x
	std::vector<double> py_line11_x(13);
	//line11 	point z			 _x
	std::vector<double> pz_line11_x(13);
	//line12		point y		 _x
	std::vector<double> py_line12_x(11);
	//line12		point z		 _x
	std::vector<double> pz_line12_x(11);
	//line13		point y		 _x
	std::vector<double> py_line13_x(5);
	//line13		point z		 _x
	std::vector<double> pz_line13_x(5);
	//line14		point y		 _x
	std::vector<double> py_line14_x(10);
	//line14		point z		 _x
	std::vector<double> pz_line14_x(10);
	//line15		point y		 _x
	std::vector<double> py_line15_x(6);
	//line15		point z		 _x
	std::vector<double> pz_line15_x(6);

	string file_name[12] = { "line1.txt", "line2.txt", "line3.txt", "line4.txt", "line5.txt", "line6.txt", "line7.txt", "line8.txt", "line9.txt", "line10.txt", "line11.txt", "line12.txt" };



	GetPointData("line1.txt", py_line1_x, pz_line1_x);
	GetPointData("line2.txt", py_line2_x, pz_line2_x);
	GetPointData("line3.txt", py_line3_x, pz_line3_x);
	GetPointData("line4.txt", py_line4_x, pz_line4_x);
	GetPointData("line5.txt", py_line5_x, pz_line5_x);
	GetPointData("line6.txt", py_line6_x, pz_line6_x);
	GetPointData("line7.txt", py_line7_x, pz_line7_x);
	GetPointData("line8.txt", py_line8_x, pz_line8_x);
	GetPointData("line9.txt", py_line9_x, pz_line9_x);
	GetPointData("line10.txt", py_line10_x, pz_line10_x);
	GetPointData("line11.txt", py_line11_x, pz_line11_x);
	GetPointData("line12.txt", py_line12_x, pz_line12_x);
	GetPointData("line13.txt", py_line13_x, pz_line13_x);
	GetPointData("line14.txt", py_line14_x, pz_line14_x);
	GetPointData("line15.txt", py_line15_x, pz_line15_x);

	/*
	//line1读取数据
	ifstream file_read1("line1.txt");
	
	string line1 = "";
	vector<string> temp1;

	if (file_read1) // 有该文件  
	{
		int j = 0;
		while (getline(file_read1, line1)) // line中不包括每行的换行符  
		{
			
			boost::split(temp1, line1, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp1.size() - 1; i++)
			{
				py_line1[j] = stod(temp1[0]);
				pz_line1[j] = stod(temp1[1]);
				j++;
			}
			
			
		}
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}

	//line2读取数据
	ifstream file_read2("line2.txt");

	string line2 = "";
	vector<string> temp2;

	if (file_read2) // 有该文件  
	{
		int j = 0;
		while (getline(file_read2, line2)) // line中不包括每行的换行符  
		{
			
			boost::split(temp2, line2, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp2.size() - 1; i++)
			{
				py_line2[j] = stod(temp2[0]);
				pz_line2[j] = stod(temp2[1]);
				j++;
			}
			

		}
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}
	//line3读取数据
	ifstream file_read3("line3.txt");

	string line3 = "";
	vector<string> temp3;

	if (file_read3) // 有该文件  
	{
		int j = 0;
		while (getline(file_read3, line3)) // line中不包括每行的换行符  
		{
			
			boost::split(temp3, line3, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp3.size() - 1; i++)
			{
				py_line3[j] = stod(temp3[0]);
				pz_line3[j] = stod(temp3[1]);
				j++;
			}
			

		}
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}
	//line4读取数据
	ifstream file_read4("line4.txt");

	string line4 = "";
	vector<string> temp4;

	if (file_read4) // 有该文件  
	{
		int j = 0;
		while (getline(file_read4, line4)) // line中不包括每行的换行符  
		{
			
			boost::split(temp4, line4, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp4.size() - 1; i++)
			{
				py_line4[j] = stod(temp4[0]);
				pz_line4[j] = stod(temp4[1]);
				j++;
			}
			

		}
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}
	//line5读取数据
	ifstream file_read5("line5.txt");

	string line5 = "";
	vector<string> temp5;

	if (file_read5) // 有该文件  
	{
		int j = 0;
		while (getline(file_read5, line5)) // line中不包括每行的换行符  
		{
			
			boost::split(temp5, line5, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp5.size() - 1; i++)
			{
				py_line5[j] = stod(temp5[0]);
				pz_line5[j] = stod(temp5[1]);
				j++;
			}
			
		}
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}
	//line6读取数据
	ifstream file_read6("line6.txt");

	string line6 = "";
	vector<string> temp6;

	if (file_read6) // 有该文件  
	{
		int j = 0;
		while (getline(file_read6, line6)) // line中不包括每行的换行符  
		{
			
			boost::split(temp6, line6, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp6.size() - 1; i++)
			{
				py_line6[j] = stod(temp6[0]);
				pz_line6[j] = stod(temp6[1]);
				j++;
			}
			

		}
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}
	//line7读取数据
	ifstream file_read7("line7.txt");

	string line7 = "";
	vector<string> temp7;

	if (file_read7) // 有该文件  
	{
		int j = 0;
		while (getline(file_read7, line7)) // line中不包括每行的换行符  
		{
			
			boost::split(temp7, line7, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp7.size() - 1; i++)
			{
				py_line7[j] = stod(temp7[0]);
				pz_line7[j] = stod(temp7[1]);
				j++;
			}
			

		}
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}

	//line8读取数据
	ifstream file_read8("line8.txt");

	string line8 = "";
	vector<string> temp8;

	if (file_read8) // 有该文件  
	{
		int j = 0;
		while (getline(file_read8, line8)) // line中不包括每行的换行符  
		{
			
			boost::split(temp8, line8, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp8.size() - 1; i++)
			{
				py_line8[j] = stod(temp8[0]);
				pz_line8[j] = stod(temp8[1]);
				j++;
			}
		

		}
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}
	//line9读取数据
	ifstream file_read9("line9.txt");

	string line9 = "";
	vector<string> temp9;

	if (file_read9) // 有该文件  
	{
		int j = 0;
		while (getline(file_read9, line9)) // line中不包括每行的换行符  
		{
			
			boost::split(temp9, line9, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp9.size() - 1; i++)
			{
				py_line9[j] = stod(temp9[0]);
				pz_line9[j] = stod(temp9[1]);
				j++;
			}

			

		}
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}
	//line10读取数据
	ifstream file_read10("line10.txt");

	string line10 = "";
	vector<string> temp10;

	if (file_read10) // 有该文件  
	{
		int j = 0;
		while (getline(file_read10, line10)) // line中不包括每行的换行符  
		{
			
			boost::split(temp10, line10, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp10.size() - 1; i++)
			{
				py_line10[j] = stod(temp10[0]);
				pz_line10[j] = stod(temp10[1]);
				j++;
			}
		

		}
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}
	//line11读取数据
	ifstream file_read11("line11.txt");

	string line11 = "";
	vector<string> temp11;

	if (file_read11) // 有该文件  
	{
		int j = 0;
		while (getline(file_read11, line11)) // line中不包括每行的换行符  
		{
			
			boost::split(temp11, line11, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp11.size() - 1; i++)
			{
				py_line11[j] = stod(temp11[0]);
				pz_line11[j] = stod(temp11[1]);
				j++;
			}
		

		}
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}
	//line12读取数据
	ifstream file_read12("line12.txt");

	string line12 = "";
	vector<string> temp12;

	if (file_read12) // 有该文件  
	{
		int j = 0;
		while (getline(file_read12, line12)) // line中不包括每行的换行符  
		{
			
			boost::split(temp12, line12, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp12.size() - 1; i++)
			{
				py_line12[j] = stod(temp12[0]);
				pz_line12[j] = stod(temp12[1]);
				j++;
			}
			

		}
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}
	//line12读取数据
	ifstream file_read13("line13.txt");

	string line13 = "";
	vector<string> temp13;

	if (file_read13) // 有该文件  
	{
		int j = 0;
		while (getline(file_read13, line13)) // line中不包括每行的换行符  
		{

			boost::split(temp13, line13, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp13.size() - 1; i++)
			{
				py_line13[j] = stod(temp13[0]);
				pz_line13[j] = stod(temp13[1]);
				j++;
			}
		

		}
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}
	//line12读取数据
	ifstream file_read14("line14.txt");

	string line14 = "";
	vector<string> temp14;

	if (file_read14) // 有该文件  
	{
		int j = 0;
		while (getline(file_read14, line14)) // line中不包括每行的换行符  
		{

			boost::split(temp14, line14, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp14.size() - 1; i++)
			{
				py_line14[j] = stod(temp14[0]);
				pz_line14[j] = stod(temp14[1]);
				j++;
			}
			

		}
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}
	//line12读取数据
	ifstream file_read15("line15.txt");

	string line15 = "";
	vector<string> temp15;

	if (file_read15) // 有该文件  
	{
		int j = 0;
		while (getline(file_read15, line15)) // line中不包括每行的换行符  
		{

			boost::split(temp15, line15, boost::is_any_of("\t"), boost::token_compress_on);
			for (int i = 0; i < temp15.size() - 1; i++)
			{
				py_line15[j] = stod(temp15[0]);
				pz_line15[j] = stod(temp15[1]);
				j++;
			}
		
		}
	}
	else // 没有该文件  
	{
		cout << "no such file" << endl;
	}
	*/

	cout << "line1 fit result\n";
	//line1 fit operation
	LineFitLeastSquares(py_line1_x, pz_line1_x, 5, result_line1);
	cout << "line2 fit result\n";		  
	//line2 fit operation		_x		  _x
	LineFitLeastSquares(py_line2_x, pz_line2_x, 5, result_line2);
	cout << "line3 fit result\n";		  
	//line3 fit operation		_x		  _x
	LineFitLeastSquares(py_line3_x, pz_line3_x, 13, result_line3);
	cout << "line4 fit result\n";		  
	//line4 fit operation		_x		  _x
	LineFitLeastSquares(py_line4_x, pz_line4_x, 4, result_line4);
	cout << "line5 fit result\n";		  
	//line5 fit operation		_x		  _x
	LineFitLeastSquares(py_line5_x, pz_line5_x, 6, result_line5);
	cout << "line6 fit result\n";		  
	//line6 fit operation		_x		  _x
	LineFitLeastSquares(py_line6_x, pz_line6_x, 7, result_line6);
	cout << "line7 fit result\n";		  
	//line7 fit operation		_x		  _x
	LineFitLeastSquares(py_line7_x, pz_line7_x, 2, result_line7);
	cout << "line8 fit result\n";		  
	//line8 fit operation		_x		  _x
	LineFitLeastSquares(py_line8_x, pz_line8_x, 2, result_line8);
	cout << "line9 fit result\n";		  
	//line9 fit operation		_x		  _x
	LineFitLeastSquares(py_line9_x, pz_line9_x, 5, result_line9);
	cout << "line10 fit result\n";
	//line10 fit operation
	LineFitLeastSquares(py_line10_x, pz_line10_x, 13, result_line10);
	cout << "line11 fit result\n";			  
	//line11 fit operation		 _x			  _x
	LineFitLeastSquares(py_line11_x, pz_line11_x, 13, result_line11);
	cout << "line12 fit result\n";			  
	//line12 fit operation		 _x			  _x
	LineFitLeastSquares(py_line12_x, pz_line12_x, 11, result_line12);
	//line13 fit operation		 _x			  _x
	LineFitLeastSquares(py_line13_x, pz_line13_x, 5, result_line13);
	//line14 fit operation		 _x			  _x
	LineFitLeastSquares(py_line14_x, pz_line14_x, 10, result_line14);
	//line15 fit operation		 _x			  _x
	LineFitLeastSquares(py_line15_x, pz_line15_x, 6, result_line15);
	


	
	
	cout << "line1\n";
	for (int i = 0; i < 3; i++){
		cout << result_line1[i];
		cout << "\t";
	}
	cout << "\n";

	cout << "line2\n";
	for (int i = 0; i < 3; i++){
		cout << result_line2[i];
		cout << "\t";
	}
	cout << "\n";

	cout << "line1 X line2\n";
	for (int i = 0; i < 2; i++)
	{
		cout << cross_l1l2[i];
		cout << "\t";
	}
	cout << "\n";
	system("pause");
}