#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

//These write function must be defined for the serialization in FileStorage to work
static void write(FileStorage& fs, const std::string&, const MyData& x)
{
    x.write(fs);
}

int main(int ac, char** av)
{
  int index = 0;
  std::stringstream filename;
  filename<<ros::package::getPath("val_data_creation")<<"/saved_images/"<<"training_img_"<<index<<".json";

    { //write
        Mat R = Mat_<uchar>::eye(3, 3),




        fs << "R" << R;                                      // cv::Mat

        fs.release();                                       // explicit close
        cout << "Write Done." << endl;
    }

   cout << endl
        << "Tip: Open up " << filename << " with a text editor to see the serialized data." << endl;

    return 0;
}
