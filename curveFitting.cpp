#include <vtkAutoInit.h>           
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

#include <vtkSmartPointer.h>  
#include <vtkPolyData.h>  
#include <vtkPolyDataMapper.h>  
#include <vtkActor.h>  
#include <vtkRenderer.h>  
#include <vtkRenderWindow.h>  
#include <vtkRenderWindowInteractor.h>  
#include <vtkProperty.h>  
#include <vtkPoints.h>  
#include <vtkPointSource.h>  
#include <vtkSpline.h>  
#include <vtkCardinalSpline.h>  
#include <vtkParametricSpline.h>  
#include <vtkParametricFunctionSource.h>  
#include <vtkKochanekSpline.h>  
//#include <vtkSCurveSpline.h>  
int main()
{
	/*
	//1.原例子设置点的方式
	vtkSmartPointer<vtkPointSource> pointSource =
	vtkSmartPointer<vtkPointSource>::New();
	pointSource->SetNumberOfPoints(5);
	pointSource->Update();


	vtkPoints* points = pointSource->GetOutput()->GetPoints();
	*/
	//2.自己设置定点，也可以运行  
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(0, 0, 0);
	points->InsertNextPoint(0, 1, 0);
	points->InsertNextPoint(1, 1, 0);
	points->InsertNextPoint(1, 0, 0);
	points->InsertNextPoint(0, 0, 0);


	//scurve spline  
	//vtkSmartPointer<vtkSCurveSpline> xSpline =  
	//vtkSmartPointer<vtkSCurveSpline>::New();  
	//vtkSmartPointer<vtkSCurveSpline> ySpline =  
	//vtkSmartPointer<vtkSCurveSpline>::New();  
	//vtkSmartPointer<vtkSCurveSpline> zSpline =  
	//vtkSmartPointer<vtkSCurveSpline>::New();  


	//cardinal spline  
	vtkSmartPointer<vtkCardinalSpline> xSpline = vtkSmartPointer<vtkCardinalSpline>::New();
	vtkSmartPointer<vtkCardinalSpline> ySpline = vtkSmartPointer<vtkCardinalSpline>::New();
	vtkSmartPointer<vtkCardinalSpline> zSpline = vtkSmartPointer<vtkCardinalSpline>::New();




	//kochanek spline  
	//vtkSmartPointer<vtkKochanekSpline> xSpline = vtkSmartPointer<vtkKochanekSpline>::New();  
	//vtkSmartPointer<vtkKochanekSpline> ySpline = vtkSmartPointer<vtkKochanekSpline>::New();  
	//vtkSmartPointer<vtkKochanekSpline> zSpline = vtkSmartPointer<vtkKochanekSpline>::New();  


	vtkSmartPointer<vtkParametricSpline> spline =
		vtkSmartPointer<vtkParametricSpline>::New();
	spline->SetXSpline(xSpline);
	spline->SetYSpline(ySpline);
	spline->SetZSpline(zSpline);
	spline->SetPoints(points);


	vtkSmartPointer<vtkParametricFunctionSource> functionSource =
		vtkSmartPointer<vtkParametricFunctionSource>::New();
	functionSource->SetParametricFunction(spline);
	functionSource->Update();


	// Setup actor and mapper  
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(functionSource->GetOutputPort());


	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);


	// Setup render window, renderer, and interactor  
	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);
	renderer->AddActor(actor);
	renderer->SetBackground(0.5, 0.5, 0.5);

	renderWindow->SetSize(960, 640);
	renderWindow->Render();
	renderWindowInteractor->Start();


	return 0;
}
