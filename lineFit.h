#ifndef LINEFIT_H
#define LINEFIT_H

#include <math.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>  
#include <string>

#include <boost/algorithm/string/classification.hpp>  
#include <boost/algorithm/string/split.hpp>

using namespace std;

class lineFit{

public:

	lineFit();
	virtual ~lineFit();

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
	void LineFitLeastSquares(vector<double> data_x, vector<double> data_y, int data_n, double *vResult);


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
	void GetCrossPoint(double a1, double b1, double c1, double a2, double b2, double c2, double *cross_Pt);


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
	void GetPointData(string filename, vector<double> &py, vector<double> &pz);


};
#endif	//LINEFIT_H
