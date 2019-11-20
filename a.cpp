#include <iostream>
#include"cv.h"
#include"cxcore.h"
#include"highgui.h"
#include<cstdio>
#include<cmath>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "sophus/so3.h"
#include "sophus/se3.h"

#include <pcl/common/transforms.h>

#include <string>
#include <fstream>
#include <sstream>
#include <map>

using std::cout;
using std::endl;

#define TIME0 0.0
#define TIME1 0.2
#define TIME2 0.4
#define TIME3 0.6
#define TIME4 0.8

class SplineFusion{
public:
	SplineFusion(std::string path, std::string pathOut);
	//??23???
	Eigen::Isometry3d cumulativeForm(Eigen::Isometry3d T_1,Eigen::Isometry3d T_2,
									 Eigen::Isometry3d T_3,Eigen::Isometry3d T_4, double u);
	Eigen::Matrix4d myCumulativeForm(Eigen::Matrix4d T_1,Eigen::Matrix4d T_2,
                                     Eigen::Matrix4d T_3,Eigen::Matrix4d T_4, double u);
	double getUt(double t, double ti, double dt){return (t-ti)/dt;};
	Sophus::SE3 fromAtoB(Sophus::SE3 a,Sophus::SE3 b);
	void SE3Eigen2Sophus(Eigen::Isometry3d e,Sophus::SE3 & s);
	void Matrix4dEigen2Sophus(Eigen::Matrix4d e,Sophus::SE3 & s);
	void test();
	void myTest();

	bool getTime(std::string t1, std::string t2, double & time1, double & result, double & u);

private:
	Sophus::SE3 t1,t2,t3,t4;
	pcl::PointCloud<pcl::PointXYZI> after,pose,temp,out,cp_in,cp_out,cp_save;
	std::string path_;
	std::string pathOut_;
	std::map<std::string, Eigen::Matrix4d> camTime2Pose;

protected:

};

SplineFusion::SplineFusion(std::string path, std::string pathOut) {
    path_ = path;
    pathOut_ = pathOut;
    cout << "test" << endl;

    typedef std::string::size_type position;

    std::ifstream fPoses;
    fPoses.open(path_.c_str());
    while(!fPoses.eof())
    {
        std::string s;
        getline(fPoses,s);
        // std::vector<position > vPos;

        //cout << "this line is " << s << endl;
        //cout << "this line size is " << s.size() << endl;

        std::string time;
        Eigen::Vector3d t;
        Eigen::Quaterniond q;

        std::vector<position > vPosition;

        if(!s.empty() && s.size()!=0)
        {
            position pos=0;
            int i=1;
            while((pos=s.find_first_of(",",pos))!=std::string::npos)
            {
                //position=s.find_first_of(flag,position);
                //cout<<"position  "<<i<<" : "<<pos<<endl;
                vPosition.push_back(pos);
                pos++;
                i++;
            }
        }

        if(s.size() == 0){
            break;
        }

        for(size_t i = 0; i <= vPosition.size(); i++)
        {
            if(i == 0){
                time = s.substr(0, vPosition.at(i));
                //cout << "time is " << time << endl;
            }
            if(i == 1){
                position length = vPosition.at(i) - vPosition.at(i-1) -1;
                //cout << ".." << vPosition.at(i-1) << endl;
                position begin = vPosition.at(i-1)+1;
                //cout << ".." << begin << endl;
                std::string strX = s.substr(begin, length);
                //cout << "strX is " << strX << endl;
                //cout << "pos is " << pos << endl;
                t.x() = std::stod(strX);
            }
            if(i == 2){
                position length = vPosition.at(i) - vPosition.at(i-1) -1;
                position begin = vPosition.at(i-1)+1;
                std::string strY = s.substr(begin, length);
                //cout << "strY is " << strY << endl;
                t.y() = std::stod(strY);
            }
            if(i == 3){
                position length = vPosition.at(i) - vPosition.at(i-1) -1;
                position begin = vPosition.at(i-1)+1;
                std::string strZ = s.substr(begin, length);
                //cout << "strZ is " << strZ << endl;
                t.z() = std::stod(strZ);
            }
            if(i == 4){
                position length = vPosition.at(i) - vPosition.at(i-1) -1;
                position begin = vPosition.at(i-1)+1;
                std::string strQX = s.substr(begin, length);
                //cout << "strQX is " << strQX << endl;
                q.x() = std::stod(strQX);
            }
            if(i == 5){
                position length = vPosition.at(i) - vPosition.at(i-1) -1;
                position begin = vPosition.at(i-1)+1;
                std::string strQY = s.substr(begin, length);
                //cout << "strQY is " << strQY << endl;
                q.y() = std::stod(strQY);
            }
            if(i == 6){
                position length = vPosition.at(i) - vPosition.at(i-1) -1;
                position begin = vPosition.at(i-1)+1;
                std::string strQZ = s.substr(begin, length);
                //cout << "strQZ is " << strQZ << endl;
                q.z() = std::stod(strQZ);
            }
            if(i == 7){
                position length = s.size() - vPosition.at(i-1) -1;
                position begin = vPosition.at(i-1)+1;
                std::string strQW = s.substr(begin, length);
                //cout << "strQW is " << strQW << endl;
                q.w() = std::stod(strQW);
            }
        }


        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        //Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        Eigen::Matrix3d rot = q.toRotationMatrix();
        T.block<3,3>(0,0) = rot;
        T.block<3,1>(0,3) = t;

        camTime2Pose.insert(std::pair<std::string, Eigen::Matrix4d>(time, T));

    }  // end of file

}

Eigen::Isometry3d SplineFusion::cumulativeForm(Eigen::Isometry3d T_1,Eigen::Isometry3d T_2,
											   Eigen::Isometry3d T_3,Eigen::Isometry3d T_4, double u) {
	Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
	Sophus::SE3 cur;
	SE3Eigen2Sophus(T_1,t1);
	SE3Eigen2Sophus(T_2,t2);
	SE3Eigen2Sophus(T_3,t3);
	SE3Eigen2Sophus(T_4,t4);
	cur =   Sophus::SE3::exp(t1.log())*
			Sophus::SE3::exp(((5 + 3*u - 3*u*u + u*u*u) / 6)*fromAtoB(t1,t2).log())*
			Sophus::SE3::exp(((1 + 3*u + 3*u*u - 2*u*u*u) / 6)*fromAtoB(t2,t3).log())*
		    Sophus::SE3::exp(((u*u*u)/6)*fromAtoB(t3,t4).log());
	result = cur.matrix();
	return result;
	
}

Eigen::Matrix4d SplineFusion::myCumulativeForm(Eigen::Matrix4d T_1,Eigen::Matrix4d T_2,
                                               Eigen::Matrix4d T_3,Eigen::Matrix4d T_4, double u) {
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    Sophus::SE3 cur;
    Matrix4dEigen2Sophus(T_1,t1);
    Matrix4dEigen2Sophus(T_2,t2);
    Matrix4dEigen2Sophus(T_3,t3);
    Matrix4dEigen2Sophus(T_4,t4);
    cur =   Sophus::SE3::exp(t1.log())*
            Sophus::SE3::exp(((5 + 3*u - 3*u*u + u*u*u) / 6)*fromAtoB(t1,t2).log())*
            Sophus::SE3::exp(((1 + 3*u + 3*u*u - 2*u*u*u) / 6)*fromAtoB(t2,t3).log())*
            Sophus::SE3::exp(((u*u*u)/6)*fromAtoB(t3,t4).log());
    result = cur.matrix();
    return result;

}

void SplineFusion::SE3Eigen2Sophus(Eigen::Isometry3d e, Sophus::SE3 & s) {
	Sophus::SE3 t1;
	Eigen::Matrix4d temp;
	temp = e.matrix();
	Eigen::Vector3d t(temp(0,3),temp(1,3),temp(2,3));
	t1 = Sophus::SE3(e.rotation(),t);
	s = t1;
}

void  SplineFusion::Matrix4dEigen2Sophus(Eigen::Matrix4d e,Sophus::SE3 & s) {
    Sophus::SE3 t1;
    Eigen::Matrix3d rot = e.block<3,3>(0,0);
    Eigen::Vector3d tran = e.block<3,1>(0,3);

    t1 = Sophus::SE3(rot,tran);
    s = t1;
}

Sophus::SE3 SplineFusion::fromAtoB(Sophus::SE3 a, Sophus::SE3 b) {
	return Sophus::SE3(a.inverse()*b);  // Taw*Twb = Tab
}

void SplineFusion::test() {
	Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/90, Eigen::Vector3d(0,0,1)).toRotationMatrix();
	Eigen::Vector3d t(1,0,0);
	Eigen::Isometry3d temp_pose1;
	Eigen::Isometry3d temp_pose2;
	Eigen::Isometry3d temp_pose3;
	Eigen::Isometry3d temp_pose4;
	Eigen::Isometry3d temp_pose5;
	Eigen::Isometry3d temp_pose6;
	Eigen::Isometry3d temp_pose7;
	pcl::PointXYZI temp1,temp2,temp3;
	temp_pose1.setIdentity();
	// 以（0,0,1）为旋转轴, 旋转45度.
	temp_pose1.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0,0,1)).toRotationMatrix());
	temp_pose1.translate(Eigen::Vector3d(0,0,0.5));
	cout<<temp_pose1.matrix()<<endl;
	temp_pose2.setIdentity();
	temp_pose2.rotate(Eigen::AngleAxisd(1.5*M_PI/4, Eigen::Vector3d(0,.1,1)).toRotationMatrix());
	temp_pose2.translate(Eigen::Vector3d(0,0,1));
	cout<<temp_pose2.matrix()<<endl;
	temp_pose3.setIdentity();
	temp_pose3.rotate(Eigen::AngleAxisd(2.2*M_PI/4, Eigen::Vector3d(0,.3,1)).toRotationMatrix());
	temp_pose3.translate(Eigen::Vector3d(0,0,2.3));
	cout<<temp_pose3.matrix()<<endl;
	temp_pose4.setIdentity();
	temp_pose4.rotate(Eigen::AngleAxisd(3.2*M_PI/4, Eigen::Vector3d(.5,0,1)).toRotationMatrix());
	temp_pose4.translate(Eigen::Vector3d(0,0,3.7));
	cout<<temp_pose4.matrix()<<endl;
	temp_pose5.setIdentity();
	temp_pose5.rotate(Eigen::AngleAxisd(2.1*M_PI/4, Eigen::Vector3d(.2,0,1)).toRotationMatrix());
	temp_pose5.translate(Eigen::Vector3d(0,1,3.7));
	temp_pose6.setIdentity();
	temp_pose6.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0,-0.2,1)).toRotationMatrix());
	temp_pose6.translate(Eigen::Vector3d(1,1,3.7));
	temp_pose7.setIdentity();
	temp_pose7.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0.4,0,1)).toRotationMatrix());
	temp_pose7.translate(Eigen::Vector3d(1,1.5,3.7));
	Sophus::SE3 a(Eigen::AngleAxisd(0, Eigen::Vector3d(0,0,1)).toRotationMatrix(),Eigen::Vector3d(0,0,0)),
			b(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0,0,1)).toRotationMatrix(),Eigen::Vector3d(1,2,1));
	cout<<"curr: a->b:"<<endl<<fromAtoB(a,b)<<endl;
	int intens = 0;
	temp2.y = 0;
	temp2.z = 0;
	// 一共300个点
	for (double j = 0; j < 100; ++j) {
		temp2.x = j/100.0;
		temp2.y = 0;
		temp2.z = 0;
		temp2.intensity = 1;
		temp.push_back(temp2);
	}
	for (double j = 0; j < 100; ++j) {
		temp2.x = 0;
		temp2.y = j/100.0;
		temp2.intensity = 2;
		temp.push_back(temp2);
	}
	for (double j = 0; j < 100; ++j) {
		temp2.x = 0;
		temp2.y = 0;
		temp2.z = j/100.0;
		temp2.intensity = 3;
		temp.push_back(temp2);
	}
	cp_in = temp;
	// 将300个点, 每个点都分别按照8种位姿做变换
	pcl::transformPointCloud(cp_in,cp_out,temp_pose1.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose2.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose3.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose4.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose5.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose6.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose7.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::io::savePCDFile("cp_save.pcd",cp_save);
	
	
	
	for (double i = 0; i < 100; ++i) {
		Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
		tf = cumulativeForm(temp_pose1,temp_pose2,temp_pose3,temp_pose4,i/100.0);  // i/100.0可以认为是插值的目标时间
		cout<<i<<endl<<tf.matrix()<<endl;
		mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		temp1.intensity = intens;
		intens++;
		pcl::transformPointCloud(temp,out,mat);
		for (double j = 0; j < 300; ++j) {
			pose.push_back(out[j]);
		}
		// after放的应该是插值的位姿, 有100种...
		after.push_back(temp1);
	}
	for (double i = 0; i < 100; ++i) {
		Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
		tf = cumulativeForm(temp_pose2,temp_pose3,temp_pose4,temp_pose5,i/100.0);
		cout<<i<<endl<<tf.matrix()<<endl;
		mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		temp1.intensity = intens;
		intens++;
		pcl::transformPointCloud(temp,out,mat);
		for (double j = 0; j < 300; ++j) {
			pose.push_back(out[j]);
		}
		after.push_back(temp1);
	}
	for (double i = 0; i < 100; ++i) {
		Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
		tf = cumulativeForm(temp_pose3,temp_pose4,temp_pose5,temp_pose6,i/100.0);
		cout<<i<<endl<<tf.matrix()<<endl;
		mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		temp1.intensity = intens;
		intens++;
		pcl::transformPointCloud(temp,out,mat);
		for (double j = 0; j < 300; ++j) {
			pose.push_back(out[j]);
		}
		after.push_back(temp1);
	}
	for (double i = 0; i < 100; ++i) {
		Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
		tf = cumulativeForm(temp_pose4,temp_pose5,temp_pose6,temp_pose7,i/100.0);
		cout<<i<<endl<<tf.matrix()<<endl;
		mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		pcl::transformPointCloud(temp,out,mat);
		for (double j = 0; j < 300; ++j) {
			pose.push_back(out[j]);
		}
		temp1.intensity = intens;
		intens++;
		after.push_back(temp1);
	}

	pcl::io::savePCDFile("temp1.pcd",after);
	pcl::io::savePCDFile("pose.pcd",pose);
}

void SplineFusion::myTest() {

    // 因为gps时间是以0, 0.2, 0.4, 0.6, 0.8固定的

    // 遍历camTime2Pose, 从第一个到倒数第四个都可以插值的需求时间戳
    // 做一个for循环, 循环中计算需要插值的需求时间戳, 并且调用cumulativeForm函数, 将时间戳和插值位姿写进文件


    cout << camTime2Pose.size() << endl;

    std::map<std::string, Eigen::Matrix4d>::iterator iterStart = camTime2Pose.begin();
    //iterStart++;
    cout << iterStart->first << endl;
    std::map<std::string, Eigen::Matrix4d>::iterator iterEnd = camTime2Pose.end();
    iterEnd--;
    cout << iterEnd->first << endl;
    iterEnd--;
    cout << iterEnd->first << endl;
    iterEnd--;
    cout << iterEnd->first << endl;
    iterEnd--;
    cout << iterEnd->first << endl;

    //return;

    std::map<std::string, Eigen::Matrix4d>::iterator iter;
    //std::map<std::string, Eigen::Matrix4d>::iterator iterNext;

    std::map<std::string, Eigen::Matrix4d>::iterator iter0;
    std::map<std::string, Eigen::Matrix4d>::iterator iter1;
    std::map<std::string, Eigen::Matrix4d>::iterator iter2;
    std::map<std::string, Eigen::Matrix4d>::iterator iter3;


    for(iter = iterStart; iter!=iterEnd; iter++)
    {
        iter0 = iter++;
        iter1 = iter++;
        iter2 = iter++;
        iter3 = iter;

        //cout << " 0 " << iter0->first << endl;
        //cout << " 1 " << iter1->first << endl;
        //cout << " 2 " << iter2->first << endl;
        //cout << " 3 " << iter3->first << endl;
        //cout << " iter " << iter->first << endl;

        iter--;
        iter--;
        iter--;

        double time1 = 0.0;
        double resultTime = 0.0;
        double u = 0.0;

        bool suc = getTime(iter1->first, iter2->first, time1, resultTime, u);  // TODO: 检查两个时间的间隔
        if(!suc){
			iter0 = iter;
			iter1 = iter;
			iter2 = iter;
			iter3 = iter;
			cout << "check" << endl;
			continue;
        }


        Eigen::Matrix4d T0 = iter0->second;
        Eigen::Matrix4d T1 = iter1->second;
        Eigen::Matrix4d T2 = iter2->second;
        Eigen::Matrix4d T3 = iter3->second;
        //double tmp_ = resultTime - time1;
        //double u_ = tmp_ * 10.0;
        //cout << std::fixed <<  std::setprecision(19) << "time1 is " << time1 << endl;
        //cout << std::fixed <<  std::setprecision(19) << "resultTime is " << resultTime << endl;
        //cout << std::fixed <<  std::setprecision(9) << "u is " << u_ << endl;
        //return;
        Eigen::Matrix4d resultT = myCumulativeForm(T0, T1, T2, T3, u);
        Eigen::Matrix3d rotResult = resultT.block<3,3>(0,0);
        Eigen::Vector3d tranResult = resultT.block<3,1>(0,3);
        Eigen::Quaterniond quaternionResult = Eigen::Quaterniond(rotResult);
        //cout << "tran: " << tranResult << endl;
        //cout << "quad: " << quaternionResult.coeffs() << endl;

        std::ofstream foutC(pathOut_, std::ios::app);
        foutC.setf(std::ios::fixed, std::ios::floatfield);
        foutC.precision(9);
        foutC << resultTime  << ",";
        foutC.precision(6);
        foutC << tranResult.x() << ","
              << tranResult.y() << ","
              << tranResult.z() << endl;
              //<< quaternionResult.x() << ","
              //<< quaternionResult.y() << ","
              //<< quaternionResult.z() << ","
              //<< quaternionResult.w() << endl;

        //<< estimator.Vs[WINDOW_SIZE].x() << ","
        //<< estimator.Vs[WINDOW_SIZE].y() << ","
        //<< estimator.Vs[WINDOW_SIZE].z() << endl;
        foutC.close();

		iter0 = iter;
		iter1 = iter;
		iter2 = iter;
		iter3 = iter;

        //test lT1Secreturn;


    }

}

bool SplineFusion::getTime(std::string t1, std::string t2, double & time1, double & result, double & u) {
    std::string t1Sec = t1.substr(0, 10);
    std::string t2Sec = t2.substr(0, 10);
    std::string t1Nsec = t1.substr(10);
    std::string t2Nsec = t2.substr(10);
    cout << "-------------------------------------------------------------------------" << endl;
    cout << "in getTime: " << t1Sec << " " << t1Nsec  << " " << t2Sec << " " << t2Nsec  << endl;
    double lT1Sec = std::stod(t1Sec);
    double lT2Sec = std::stod(t2Sec);
    double lT1Nsec = std::stod(t1Nsec);
    double lT2Nsec = std::stod(t2Nsec);
    //cout << "in getTime: " << std::fixed <<  std::setprecision(9) << lT1Sec << " " << lT1Nsec  << " " << lT2Sec << " " << lT2Nsec  << endl;
    lT1Nsec = lT1Nsec * 1e-9;
    lT2Nsec = lT2Nsec * 1e-9;
    cout << "in getTime: " << std::fixed <<  std::setprecision(9) << lT1Sec << " " << lT1Nsec  << " " << lT2Sec << " " << lT2Nsec  << endl;

    double resultSec;
    double resultNsec;

    if(lT1Sec == lT2Sec && lT1Nsec < lT2Nsec)
    {
        if(lT1Nsec<TIME1 && TIME1<lT2Nsec){
            //cout << "1" << endl;
            resultNsec = 2.0*1e-1;
        }
        else if(lT1Nsec<TIME2 && TIME2<lT2Nsec){
            //cout << "2" << endl;
            resultNsec = 4.0*1e-1;
        }
        else if(lT1Nsec<TIME3 && TIME3<lT2Nsec){
            //cout << "3" << endl;
            resultNsec = 6.0*1e-1;
        }
        else if(lT1Nsec<TIME4 && TIME4<lT2Nsec){
            //cout << "4" << endl;
            resultNsec = 8.0*1e-1;
        }
        else {
			return false;
        }
        resultSec = lT1Sec;
    }
    if(lT1Sec < lT2Sec)
    {
        cout << "test lT1Sec: " << lT1Sec << endl;
        cout << "test lT2Sec: " << lT2Sec << endl;
        resultSec = lT2Sec;
        resultNsec = 0.0*1e-1;
    }
    time1 = lT1Sec + lT1Nsec;
    result = resultSec+resultNsec;
    double tmp = result - time1;
    u = tmp*10.0;
    cout << std::fixed <<  std::setprecision(9) << "in getTime, result is " << time1 << " " << result << " " << u <<endl;
	return true;
}

using namespace std;

int main(int argc, char *argv[])
{
	SplineFusion sf((string)argv[1], (string)argv[2]);
	sf.myTest();
	//sf.test();
	return 0;
}
