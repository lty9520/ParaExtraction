
#include <stdlib.h>
#include <time.h>

#include <GL/glut.h>

#pragma comment(lib,"glut32.lib")

//4�����Ƶ��3D���ꡪ��z����ȫΪ0
GLfloat ctrlpoints[4][3] = {
	{ -4, -4, 0 }, { -2, 4, 0 }, { 2, -4, 0 }, { 4, 4, 0 }
};

void init(void)
{
	//����ɫ
	glClearColor(0.0, 0.0, 0.0, 1.0);
	//�����Ƶ�����ӳ��Ϊ��������
	//����1��GL_MAP1_VERTEX_3��3ά������
	//����2��3�����Ʋ���t��u��ȡֵ��Χ[0, 1]
	//����4�������ڲ�ֵ���Ĳ���3��������3ά����
	//����5�����߼�Ĳ���Ϊ������4�����������ܲ���Ϊ12
	//����6�����Ƶ��ά������Ԫ�ص�ַ
	//ע��: ������������������ز�����������ctrlpoints���ݸ������߲���
	glMap1f(GL_MAP1_VERTEX_3, 0.0, 1.0, 3, 4, &ctrlpoints[0][0]);
	//�򿪿��ء�������3ά������Ƶ㵽������ת������
	glEnable(GL_MAP1_VERTEX_3);
	glShadeModel(GL_FLAT);

	//���뿪��2��ȥ����ע�ͣ������÷�����
	/*
	glEnable(GL_BLEND);
	glEnable(GL_LINE_SMOOTH);  //����ֱ�߷�����
	glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST);  // Antialias the lines
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	*/
}

void display(void)
{
	int i;
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(1.0, 1.0, 1.0);

	//���뿪��1��ȥ����ע�ͣ��鿴��̬�����߻�ͼЧ������̬���¿��Ƶ�����
	/*
	for(int t = 0; t < 4; t++) {
	for(int j = 0; j < 3; j++)
	ctrlpoints[t][j] = (rand() % 1024 / 1024.0 - 0.5) * 10;
	}
	//��̬ӳ��
	glMap1f(GL_MAP1_VERTEX_3, 0.0, 1.0, 3, 4, &ctrlpoints[0][0]);
	*/
	glLoadIdentity();
	glColor3f(1.0, 0.0, 0.0);
	//���������߶�
	glBegin(GL_LINE_STRIP);
	//����t��uȡֵΪi/30������31����
	for (i = 0; i <= 30; i++)
		glEvalCoord1f((GLfloat)i / 30.0);   //����4�����Ƶ�����Ĳ�������ֵ
	glEnd();
	/* ��ʾ���Ƶ� */
	glPointSize(5.0);
	glBegin(GL_POINTS);
	for (i = 0; i < 4; i++)
		glVertex3fv(&ctrlpoints[i][0]);
	glEnd();

	glTranslatef(-0.1f, 0.1f, 0.0f);
	glColor3f(0.0, 1.0, 0.0);
	//glLineWidth(2.0);
	//���������߶Ρ����߶���Խ�࣬����Խ�⻬
	glBegin(GL_LINE_STRIP);
	//���ò���t��uȡֵΪi/60������61����
	//ʵ�飺����t��-2�仯��+2���ɿ���ʲôЧ��
	for (i = 0; i <= 60; i++)
		glEvalCoord1f((GLfloat)i / 60.0);  //����4�����Ƶ�����Ĳ�������ֵ
	glEnd();

	glTranslatef(-0.1f, 0.1f, 0.0f);
	glColor3f(1.0, 1.0, 1.0);
	//���������߶�
	glBegin(GL_LINE_STRIP);
	//���ò���t��uȡֵΪi/60������61����
	//ʵ�飺����t��-2�仯��+2���ɿ���ʲôЧ��
	for (i = 0; i <= 100; i++)
		glEvalCoord1f((GLfloat)i / 100.0);
	glEnd();

	glutSwapBuffers();
}

//3D�ռ��л���2DЧ������������ͶӰ
void reshape(GLsizei w, GLsizei h)
{
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (w <= h)
		glOrtho(-5.0, 5.0, -5.0*(GLfloat)h / (GLfloat)w, 5.0*(GLfloat)h / (GLfloat)w, -5.0, 5.0);
	else
		glOrtho(-5.0*(GLfloat)w / (GLfloat)h, 5.0*(GLfloat)w / (GLfloat)h, -5.0, 5.0, -5.0, 5.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'x':
	case 'X':
	case 27:   //ESC��
		exit(0);
		break;
	default:
		break;
	}
}

int main(int argc, char** argv)
{
	srand((unsigned int)time(0));
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);//ʹ��˫����ģʽ����Ȼ���
	glutInitWindowSize(800, 800);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("2D Bezier����");
	init();
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(display);//���ÿ���ʱ���õĺ���
	glutMainLoop();
	return 0;
}