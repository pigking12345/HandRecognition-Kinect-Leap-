#include "DBSCAN.h"

#include "DBSCAN.h"

#include <iostream>
using namespace std; 

static bool DBSCANDataCompareVal(DBSCANData* i, DBSCANData* j){
	return ((i->val)<(j->val));  
}

static bool DBSCANDataCompareId(DBSCANData* i, DBSCANData* j){
	return ((i->id)<(j->id));  
}

DBSCANData::DBSCANData(float v){
	id = DBSCAN::s_id++; 
	val = v; 
	state = UNVISITED; 
	clustered = false; 
}

int DBSCAN::s_id = 0;

DBSCAN::DBSCAN(const vector<float>& data, float eps, int minPts){
	m_data.clear(); 
	floatVec2DBVec(data, m_data); 
	m_eps = eps; 
	m_minPts = minPts; 
}

DBSCAN::~DBSCAN(){
	for(list<DBSCANData*>::iterator it = m_data.begin(); it != m_data.end(); ++it){
		delete *it; 
		*it = NULL; 
	}
}

void DBSCAN::regionQuery(DBSCANData* P, std::list<DBSCANData*>& output){
	output.clear(); 
	DBSCANData* pi; 
	for(list<DBSCANData*>::iterator it=m_data.begin(); it != m_data.end(); it++){
		pi = *it; 
		if(abs(P->val-pi->val)<m_eps)
			output.push_back(pi); 
	}
}

void DBSCAN::expandCluster(DBSCANData* P, list<DBSCANData*>& NeighborPts, list<DBSCANData*>& C){
	P->clustered = true; 
	C.push_back(P); 

	list<DBSCANData*> NeighborPtsCopy; 

	NeighborPtsCopy.assign(NeighborPts.begin(), NeighborPts.end()); 
	NeighborPtsCopy.sort(DBSCANDataCompareId);

	for(list<DBSCANData*>::iterator it = NeighborPts.begin(); it != NeighborPts.end(); ++it){
		DBSCANData* pi = *it; 
		if(pi->state==UNVISITED){
			pi->state = VISITED; 
			list<DBSCANData*> nPts; 
			regionQuery(pi, nPts); 
			if(nPts.size()>=m_minPts){
				nPts.sort(DBSCANDataCompareId); 
				list<DBSCANData*>::iterator nPtIt = nPts.begin(); 
				for(list<DBSCANData*>::iterator it2=NeighborPtsCopy.begin(); it2!=NeighborPtsCopy.end(); it2++){
					while(nPtIt!=nPts.end()&&(*nPtIt)->id>(*it2)->id){
						nPtIt++;
					}
					while(nPtIt!=nPts.end()&&(*nPtIt)->id<=(*it2)->id){
						if((*nPtIt)->id<(*it2)->id){
							NeighborPts.insert(NeighborPts.end(), *nPtIt);
							NeighborPtsCopy.insert(it2, *nPtIt); 
						}
						nPtIt++;
					}
					if(nPtIt==nPts.end()){
						break; 
					}
				}
			}
		}
		if(pi->clustered==false){
			pi->clustered = true; 
			C.push_back(pi); 
		}
	}
}

void DBSCAN::clustering(vector<float>& output){
	list<list<DBSCANData*>> clusters; 
	clusters.clear(); 
	m_data.sort(DBSCANDataCompareVal); 
	for(list<DBSCANData*>::iterator it=m_data.begin(); it!=m_data.end(); it++){
		DBSCANData* P = *it; 
		if(P->state==UNVISITED){
			P->state = VISITED; 
			list<DBSCANData*> NeighborPts;
			regionQuery(P, NeighborPts); 
			if(NeighborPts.size()<m_minPts){
				P->state = NOISE; 
			}
			else{
				clusters.push_back(list<DBSCANData*>()); 
				expandCluster(P, NeighborPts, clusters.back()); 
			}
		}

	}

	int maxSize = 0; 
	list<list<DBSCANData*>>::iterator maxIt; 
	cout<<"clusters size"<<clusters.size()<<endl; 
	for (list<list<DBSCANData*>>::iterator it=clusters.begin(); it!=clusters.end(); it++){
		cout<<"cluster size: "<<it->size()<<endl; 
		if(it->size()>maxSize){
			maxSize = it->size(); 
			maxIt = it; 
		}
	}
	DBVec2FloatVec(*maxIt, output); 
}

void DBSCAN::floatVec2DBVec(const vector<float>& input, list<DBSCANData*>& output){
	output.clear(); 
	for(int i=0, vecSize=(int)input.size(); i<vecSize; i++){
		output.push_back(new DBSCANData(input[i])); 
	}
}

void DBSCAN::DBVec2FloatVec(list<DBSCANData*>& input, vector<float>& output){
	output.clear(); 
	for(list<DBSCANData*>::iterator it = input.begin(); it!= input.end(); it++){
		output.push_back((*it)->val); 
	}
}