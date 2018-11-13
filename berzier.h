#ifndef BERZIER_H
#define BERZIER_H

#include <iostream>  
#include <vector>  
#include <cmath>  
#include <fstream>
#include <cassert>
#include <string>

#define POINTNUM 6
double Em[POINTNUM][4];

using namespace std;

class berzier_fit{


private:
	


public:

	berzier_fit();
	virtual ~berzier_fit();


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
	double calcR(vector<double> vy, vector<double> vx, int n, double coefficient[]);
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
	double sum(vector<double> Vnum, int n);
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
	double MutilSum(vector<double> Vx, vector<double> Vy, int n);
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
	double RelatePow(vector<double> Vx, int n, int ex);
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
	double RelateMutiXY(vector<double> Vx, vector<double> Vy, int n, int ex);
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
	void EMatrix(vector<double> Vx, vector<double> Vy, int n, int ex, double coefficient[]);
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
	void CalEquation(int exp, double coefficient[]);
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
	double F(double c[], int l, int m);
	//��ȡ�ļ�������Ϸ���
	//************************************
	// Method:    readTxt
	// FullName:  berzier_fit::readTxt
	// Access:    public 
	// Returns:   void
	// Qualifier:
	// Parameter: string file
	//************************************
	void readTxt(string file);
};
#endif	//BERZIER_H
