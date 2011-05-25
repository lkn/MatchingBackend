#ifndef SURFMATCHER_H__
#define SURFMATHCER_H__

#include <opencv/cv.h>
#include "../support/Logger.h"

// <KeyPoints, Descriptors>
typedef pair<CvSeq *, CvSeq *> SURFFeatures_t;

static CvScalar colors[] = 
{
    {{0,0,255}},
    {{0,128,255}},
    {{0,255,255}},
    {{0,255,0}},
    {{255,128,0}},
    {{255,255,0}},
    {{255,0,0}},
    {{255,0,255}},
    {{255,255,255}}
};

typedef struct {
	int id;
	string name;
	string path;
	string description;
	IplImage *image;
} ImageData;

typedef struct {
	double hessian_threshold;
	int extended_parameter;
	int image_width;
	int image_height;
} QueryParams;

typedef struct {
	double hessian_threshold;
	int extended_parameter;
	int image_width;
	int image_height;
	double match_threshold;
	string db_path;
} SURFMatcherParams;

class SURFMatcher {
public:
	SURFMatcher(Logger *logger, const SURFMatcherParams& params);
	~SURFMatcher();

	int BuildFromXml(std::string fileName);
	// list of (name, path)
	int BuildFromList(vector<pair<string, string> > pathTuples);

	// Finds matching keypoints and estimates an accuracy
	double FindPairs(const CvSeq* objectKeypoints,
		const CvSeq* objectDescriptors, const CvSeq* imageKeypoints,
		const CvSeq* imageDescriptors, vector<int>& ptpairs) const;

	double FlannFindPairs(const CvSeq*, const CvSeq* objectDescriptors,
           const CvSeq*, const CvSeq* imageDescriptors, vector<int>& ptpairs) const;

	string MatchAgainstLibrary(const char *queryImageName, 
		const IplImage *queryImage, const CvSeq *queryKeyPoints, const CvSeq *queryDescriptors) const;

	int NaiveNearestNeighbor(const float* vec,
		int laplacian, const CvSeq* model_keypoints, const CvSeq* model_descriptors) const;

	void Visualize(string dirName,
		string queryImageName, IplImage *queryImage,
		CvSeq *queryKeyPoints, CvSeq *queryDescriptors) const;

	int LocatePlanarObject(const CvSeq* objectKeypoints,
		const CvSeq* objectDescriptors, const CvSeq* imageKeypoints, 
		const CvSeq* imageDescriptors, const vector<int>& ptpairs, 
		const CvPoint src_corners[4], CvPoint dst_corners[4]) const;

 	void ConnectMatchingFeatures(IplImage *correspond,
		const CvSeq *objectKeypoints, const CvSeq *imageKeypoints,
		const vector<int>& ptpairs, int objectHeight) const;

	//const vector<IplImage *>& referenceImages() const { return referenceImages_; }
	//const vector<string>& referenceImageNames() const { return referenceImageNames_; }
	//const std::vector<SURFFeatures_t>& refKD() const { return refKD_; }

protected:
	double CompareSURFDescriptors(const float *d1, const float *d2, double best, int length) const;
	void Extract();

private:
	std::vector<SURFFeatures_t> refKD_;  // keypoints + descriptors
	//std::vector<std::string> referenceImageNames_;
	//std::vector<IplImage *> referenceImages_;
	std::vector<ImageData *> referenceData_;

	Logger *logger_;
	SURFMatcherParams params_;
	CvMemStorage *storage_;
};

#endif