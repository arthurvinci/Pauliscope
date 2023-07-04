//
// Created by arthur on 25/03/2022.
//

#ifndef DEFS_HPP
#define DEFS_HPP

#define ACCEPTABLE_ERROR 0.01

#define abs(x) x >=0 ? x : -x

#define min3(a,b,c)  (((a) < (b))? (((a) < (c))? (a) : (c)) : (((b) < (c))? (b) : (c)))

#define max3(a,b,c) (((a) > (b))? (((a) > (c))? (a) : (c)) : (((b) > (c))? (b) : (c)))

#define GREEN_COLOR 0, 102, 0

#define RED_COLOR 255, 0, 0

#endif //DEFS_HPP
