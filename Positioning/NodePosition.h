#ifndef NodePostion_h
#define NodePostion_h

typedef struct ShelfShape{
    float BottomEdge;
    float WaistEdge;
}ShelfShape;

typedef struct Pos3D{
    float x;
    float y;
    float z;
}Pos3D;

//configure the shape of shlef which used to construct reference
int shelf_shape_config(ShelfShape shape);

int nodes_position_calc(RefNode *pRfNds,int max_nd_num,bool conf_data_from_stdin);
#endif

