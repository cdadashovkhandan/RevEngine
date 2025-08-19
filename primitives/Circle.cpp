// #include "Circle.h"
// #include <qdebug.h>

// Circle::Circle() {
//     shapeType = PrimitiveType::CIRCLE;
// }


// std::vector<ParamPair> Circle::buildParameters(const std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points, const float maxMagnitude) const
// {
//     // Matlab reference

//     /*
//         function [A,B]=ellisse(xyz)

//             t=0:(2*pi)/1000:2*pi;
//             A=zeros(size(xyz,1),numel(t));
//             B=zeros(size(xyz,1),numel(t));
//             for i=1:size(xyz,1)
//                 for j=1:numel(t)
//                       if (abs(sin(t(j)))>0.00000000001 && abs(cos(t(j)))>0.00000000001)
//                             mat=[cos(t(j)) 0; 0 sin(t(j))];
//                             sol=pinv(mat)*xyz(i,:)';
//                             A(i,j)=sol(1);
//                             B(i,j)=sol(2);
//                       end
//                 end
//             end
//             col=find(sum(A));
//             A=A(:,col);
//             B=B(:,col);
//             col=find(sum(B));
//             A=A(:,col);
//             B=B(:,col);
//         end
// */

//     qDebug() << "Building Parameters for Hough Transform...";


//     float eps = 0.00000000001f; // TODO: better way to write this?

//     float step = 2 * M_PI / 1000;

//     std::vector<ParamPair> output;

// #pragma omp parallel for
//     for (pcl::PointXYZ point : points)
//     {
//         for (float t = 0; t < 2 * M_PI; t += step)
//         {
//             float cos_t = qCos(t);
//             float sin_t = qSin(t);
//             if (cos_t > eps && sin_t > eps)
//             {
//                 Eigen::Matrix2f mat {
//                     {cos_t, 0},
//                     {0, sin_t}
//                 };
//                 Eigen::Vector2f result = mat.completeOrthogonalDecomposition().pseudoInverse() * point.getVector2fMap();


//                 //TODO: the original implementation put these into NxM matrices to deduce index. Maybe this should too.
//                 std::vector<float> params({result[0], result[1]});
//                 output.push_back(ParamPair(pcl::PointIndices::Ptr(new pcl::PointIndices), params));
//             }
//         }
//     }

//     qDebug() << "Parameters built: " << output.size();
//     return output;
// }




// bool Circle::isIntersecting(const pcl::PointXYZ point, const std::vector<float> params, const float maxMagnitude) const
// {
//     // Matlab reference
//     /*
//         function  [coord, maxCoord, Ps]= extract_curve(curveId, xy, PtMin, PtMax, Eps)
//             a=PtMin(1):Eps(1):PtMax(1);
//             Na=numel(a);
//             if numel(Eps)>1
//                 b=PtMin(2):Eps(2):PtMax(2);
//                 Nb=numel(b);
//                 H=zeros(Na,Nb);
//                 [A,B]=HT(curveId,xy,a);
//                 for i=1:size(A,1)
//                     for j=1:Na
//                         for k=1:Nb
//                             aus=1;
//                             for t=1:size(A,2)
//                                 if (a(j)<=A(i,t) && A(i,t)<(a(j)+Eps(1))) && (b(k)<=B(i,t) && B(i,t)<(b(k)+Eps(2)) && aus)
//                                     H(j,k)=H(j,k)+1;
//                                     aus=0;
//                                 end
//                             end
//                         end
//                     end
//                 end
//                 [row,col]=find(H == max(max(H)));
//                 aCoord=a(row);
//                 bCoord=b(col);
//                 coord=[aCoord, bCoord];
//                 maxCoord=max(max(H));

//                  p=(60/100)*maxCoord;
//                 Ps=zeros(size(xy));
//                 for i=1:size(A,1)
//                     for j=1:Na
//                         for k=1:Nb
//                             for t=1:size(A,2)
//                                 if (a(j)<=A(i,t) && A(i,t)<(a(j)+Eps(1))) && (b(k)<=B(i,t) && B(i,t)<(b(k)+Eps(2)))
//                                     if H(j,k)>p
//                                         Ps(i,:)=xy(i,:);
//                                     end
//                                 end
//                             end
//                         end
//                     end
//                 end
//             else
//                 H=zeros(Na,1);
//                 [A,B]=HT(curveId,xy,a);
//                 for i=1:size(A,1)
//                     for j=1:Na
//                         aus=1;
//                         for t=1:size(A,2)
//                             if (a(j)<=A(i,t) && A(i,t)<(a(j)+Eps(1)) && aus)
//                                 H(j)=H(j)+1;
//                                 aus=0;
//                             end
//                         end
//                     end
//                  end
//                 [row]=find(H == max(H));
//                 coord=a(row);
//                 maxCoord=max(H);

//                 p=(60/100)*maxCoord;
//                 Ps=zeros(size(xy));
//                 for i=1:size(A,1)
//                     for j=1:Na
//                         for t=1:size(A,2)
//                             if (a(j)<=A(i,t) && A(i,t)<(a(j)+Eps(1)))
//                                 if H(j)>p
//                                    Ps(i,:)=xy(i,:);
//                                 end
//                             end
//                         end
//                     end
//                 end
//             end

//             Ps=Ps(find(sum(Ps')),:);

//         end
// */



//     // This should somehow account for limited parameter ranges. Though maybe actually BuildParamereterss should be the one responding.
// }






