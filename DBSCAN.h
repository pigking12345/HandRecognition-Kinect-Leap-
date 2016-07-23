#pragma once
#ifndef DBSCAN_H
#define DBSCAN_H
#include <vector>
#include <list>
#include <algorithm>

enum DBSCANState{
	UNVISITED, 
	VISITED, 
	NOISE
};

struct DBSCANData{
	int id; 
	float val; 
	DBSCANState state; 
	bool clustered; 

	DBSCANData(float v); 
};

class DBSCAN{
public:
	DBSCAN(const std::vector<float>& data, float eps, int minPts);
	~DBSCAN(); 

	void clustering(std::vector<float>& output); 

	void expandCluster(DBSCANData* P, std::list<DBSCANData*>& NeighborPts, std::list<DBSCANData*>& C); 

	void regionQuery(DBSCANData* P, std::list<DBSCANData*>& output); 

	void floatVec2DBVec(const std::vector<float>& input, std::list<DBSCANData*>& output); 
	void DBVec2FloatVec(std::list<DBSCANData*>& input, std::vector<float>& output); 

	static int	s_id; 
private:
	std::list<DBSCANData*>		m_data; 
	float						m_eps; 
	int							m_minPts; 
};

#endif
