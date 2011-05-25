// So we have one query image and several images in a library image. For each keypoint descriptor of query image
// the one nearest descriptor is found the entire collection of library images. As an extra, to aid in debugging,
// we can visualize the result of matching by saving resultant match images... each of which combines query and
// library image with matches between them (if they exist). The match is drawn as a line between corresponding
// points.Count of all matches is equal to count of query keypoints.
//
// TODO: use FAST
#include "stdafx.h"

#include <fstream>
#include <iostream>

#include <opencv/cv.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "thirdparty/tinyxml/tinyxml.h"

#include "support/cv_helper.h"
#include "support/util.h"
#include "support/Logger.h"
#include "matching/SURFMatcher.h"

using namespace cv;
using namespace std;

// TODO: change the fact that query images must be smaller than training image
const string settingsFilename = "settings_match_image_exe.xml";


bool readQuery(const string& queryImageName, IplImage **queryImage, int width, int height) {
    cout << "< Reading the images..." << endl;
	
	IplImage *tmp = cvLoadImage(queryImageName.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    if (!tmp) {
        cerr << "Query image can not be read." << endl << ">" << endl;
        return false;
    }

	*queryImage = resizeImage(tmp, width, height, true);

    cvReleaseImage(&tmp);

    cout << ">" << endl;

    return true;
}

void readQueryFilenames(const string& filename, vector<string>& queryFilenames) {
    const char dlmtr = '/';

    queryFilenames.clear();

    ifstream file(filename.c_str());
    if (!file.is_open())
        return;

    size_t pos = filename.rfind(dlmtr);
    string dirName = pos == string::npos ? "" : filename.substr(0, pos) + dlmtr;
    while (!file.eof()) {
        string str;
		getline(file, str);
        if (str.empty()) break;
        queryFilenames.push_back(dirName + str);
    }
    file.close();
}

bool readSettingsFile(const string& settingsFilename, string& logName, string& fileWithQueryImages, string& fileWithLibraryImages,
	QueryParams& queryParams, SURFMatcherParams& libraryParams) {
	
	if (settingsFilename.empty()) return false;
	const char *tmp = NULL;
	TiXmlDocument *doc = new TiXmlDocument(settingsFilename.c_str());
	doc->LoadFile();
	TiXmlElement *settingsElement = doc->FirstChildElement("settings");
	TiXmlElement *queryElement = settingsElement->FirstChildElement("query");
	if ((tmp = queryElement->Attribute("path")) == NULL) {
		cerr << "No query image file!" << endl;
	} else {
		fileWithQueryImages = string(tmp);
	}

	TiXmlElement *surfElement = queryElement->FirstChildElement("surf");
	queryParams.hessian_threshold = atof(surfElement->FirstChildElement("hessian_threshold")->FirstChild()->ToText()->Value());
	queryParams.extended_parameter = atoi(surfElement->FirstChildElement("extended_parameter")->FirstChild()->ToText()->Value());

	TiXmlElement *imageElement = queryElement->FirstChildElement("image");
	queryParams.image_width = atoi(imageElement->FirstChildElement("width")->FirstChild()->ToText()->Value());
	queryParams.image_height = atoi(imageElement->FirstChildElement("height")->FirstChild()->ToText()->Value());

	TiXmlElement *libraryElement = settingsElement->FirstChildElement("library");
	if ((tmp = libraryElement->Attribute("path")) == NULL) {
		cerr << "No library file!" << endl;
	} else {
		fileWithLibraryImages = string(tmp);
	}

	surfElement = libraryElement->FirstChildElement("surf");
	libraryParams.hessian_threshold = atof(surfElement->FirstChildElement("hessian_threshold")->FirstChild()->ToText()->Value());
	libraryParams.extended_parameter = atoi(surfElement->FirstChildElement("extended_parameter")->FirstChild()->ToText()->Value());

	imageElement = libraryElement->FirstChildElement("image");
	libraryParams.image_width = atoi(imageElement->FirstChildElement("width")->FirstChild()->ToText()->Value());
	libraryParams.image_height = atoi(imageElement->FirstChildElement("height")->FirstChild()->ToText()->Value());

	libraryParams.match_threshold = atof(libraryElement->FirstChildElement("match_threshold")->FirstChild()->ToText()->Value());
	
	const char *name = settingsElement->FirstChildElement("log")->Attribute("name");
	if (name != NULL) {
		logName = string(name);
	}
	return true;
}

/*
int _tmain(int argc, char** argv) {

	SURFMatcherParams libraryParams;
	QueryParams queryParams;
	string logName, fileWithQueryImages, fileWithLibraryImages;

	if (!readSettingsFile(settingsFilename, logName, fileWithQueryImages, fileWithLibraryImages, queryParams, libraryParams)) {
		fprintf(stderr, "hey man, where's the setting file %s!\n", settingsFilename.c_str());
		return -1;
	}

	Logger logger(logName);
    IplImage *queryImage;

	vector<string> queryImageFilenames;
	readQueryFilenames(fileWithQueryImages, queryImageFilenames);

	SURFMatcher surfMatcher(&logger, libraryParams);
	surfMatcher.BuildFromXml(fileWithLibraryImages);

	CvMemStorage *storage = cvCreateMemStorage(0);
	CvSURFParams params = cvSURFParams(queryParams.hessian_threshold, queryParams.extended_parameter);

	vector<string>::const_iterator it;
	for (it = queryImageFilenames.begin(); it != queryImageFilenames.end(); ++it) {
		if (!readQuery(it->c_str(), &queryImage, queryParams.image_width, queryParams.image_height)) {
	        return -1;
	    }

		CvSeq *queryKeyPoints = 0, *queryDescriptors = 0;
		double tt = (double) -cvGetTickCount();
		cvExtractSURF(queryImage, 0, &queryKeyPoints, &queryDescriptors, storage, params);
		logger.Log(INFO, "%s - Query Descriptors %d\nQuery Extraction Time = %gm\n",
			it->c_str(), queryDescriptors->total,
			((tt + cvGetTickCount()) / cvGetTickFrequency()*1000.));

		
		string res = surfMatcher.MatchAgainstLibrary(it->c_str(), queryImage, queryKeyPoints, queryDescriptors);
		//if (!res.empty()) {
		//	surfMatcher.Visualize(dirToSaveResImages, it->c_str(), queryImage, queryKeyPoints, queryDescriptors);
		//}

		// TOADD
		cvReleaseImage(&queryImage);
	}
	return 0;
}
*/