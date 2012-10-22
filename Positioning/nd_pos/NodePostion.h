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

int node_position_calc(PSample RawData,RefNode &RfNds[max_rx_num]);

#endif

