// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Principal.h"

#include "Homogeneous.h"
#include "Timer.h"
#include "Stat.h"
#include "SGrid.h"

namespace hh {

namespace {

inline void rotate(float& v1, float& v2, float tau, float vsin) {
    float t1 = v1, t2 = v2;
    v1 -= vsin*(t2+t1*tau);
    v2 += vsin*(t1-t2*tau);
}

void principal_components(CArrayView<Vec3<float>> va, const Vec3<float>& avgp, Frame& f, Vec3<float>& eimag) {
    // Note that this builds on version of compute_eigenvectors() specialized to n = 3.
    const int n = 3;
    SGrid<float, n, n> a;
    for_int(c0, n) {
        for_int(c1, c0+1) {
            double sum = 0.; for_int(i, va.num()) { sum += (va[i][c0]-avgp[c0])*(va[i][c1]-avgp[c1]); }
            a[c0][c1] = a[c1][c0] = float(sum/va.num());
        }
    }
    Vec<float,n> val; for_int(i, n) { val[i] = a[i][i]; }
    SGrid<float, n, n> vec = { {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f} };
    for_int(iter, INT_MAX) {
        {
            float sum = 0.f; for_int(i, n-1) for_intL(j, i+1, n) sum += abs(a[i][j]);
            if (!sum) break;
        }
        for_int(i, n-1) {
            for_intL(j, i+1, n) {
                float thresh = 1e2f*abs(a[i][j]);
                if (abs(val[i])+thresh==abs(val[i]) &&
                    abs(val[j])+thresh==abs(val[j])) {
                    a[i][j] = 0.f;
                } else if (abs(a[i][j])>0.f) {
                    float vtan; {
                        float dd = val[j]-val[i];
                        if (abs(dd)+thresh==abs(dd)) {
                            vtan = a[i][j]/dd;
                        } else {
                            float theta = 0.5f*dd/a[i][j];
                            vtan = 1.f/(abs(theta)+sqrt(1.f+square(theta)));
                            if (theta<0.f) vtan = -vtan;
                        }
                    }
                    float vcos = 1.f/sqrt(1.f+square(vtan)), vsin = vtan*vcos;
                    float tau = vsin/(1.f+vcos);
                    val[i] -= vtan*a[i][j];
                    val[j] += vtan*a[i][j];
                    a[i][j] = 0.f;
                    for_int(k, i)       { rotate(a[k][i], a[k][j], tau, vsin); }
                    for_intL(k, i+1, j) { rotate(a[i][k], a[k][j], tau, vsin); }
                    for_intL(k, j+1, n) { rotate(a[i][k], a[j][k], tau, vsin); }
                    for_int(k, n)       { rotate(vec[i][k], vec[j][k], tau, vsin); }
                }
            }
        }
        assertx(iter<10);
    }
    // Insertion sort: eigenvalues in descending order.
    for_int(i, n) {
        int imax = i;
        float vmax = val[i];
        for_intL(j, i+1, n) { if (val[j]>=vmax) { imax = j; vmax = val[j]; } }
        if (imax==i) continue;
        std::swap(val[i], val[imax]);
        swap_ranges(vec[i], vec[imax]);
    }
    for_int(i, n) {
        float v = val[i];
        if (v<0.f) v = 0.f;     // for numerics
        v = sqrt(v);
        eimag[i] = v;
        if (!v) v = 1e-15f;     // very small but non-zero vector
        f.v(i) = v*vec[i];
    }
    f.p() = Point(avgp[0], avgp[1], avgp[2]);
    f.make_right_handed();
}

} // namespace

void principal_components(CArrayView<Point> pa, Frame& f, Vec3<float>& eimag) {
    assertx(pa.num()>0);
    Homogeneous hp; for_int(i, pa.num()) { hp += pa[i]; }
    Point avgp = to_Point(hp/float(pa.num()));
    principal_components(CArrayView<Vec3<float>>(pa.data(), pa.num()), avgp, f, eimag);
}

void principal_components(CArrayView<Vector>va, Frame& f, Vec3<float>& eimag) {
    assertx(va.num()>0);
    principal_components(CArrayView<Vec3<float>>(va.data(), va.num()), Point(0.f, 0.f, 0.f), f, eimag);
}

void subtract_mean(MatrixView<float> mi) {
    const int m = mi.ysize(), n = mi.xsize(); assertx(m>=2 && n>0);
    for_int(j, n) {
        double sum = 0.; for_int(i, m) sum += mi[i][j]; // sum across columns
        float avg = float(sum/m);
        for_int(i, m) { mi[i][j] -= avg; }
    }
}

// Compute eigenvectors of a symmetric matrix.
static void compute_eigenvectors(MatrixView<float> a, MatrixView<float> mo, ArrayView<float> eimag) {
    // See also the specialized version in principal_components() above, for n==3.
    const int n = a.ysize();
    assertx(a.xsize()==n && mo.ysize()==n && mo.xsize()==n && eimag.num()==n);
    for_int(i, n) for_int(j, n) { mo[i][j] = i==j ? 1.f : 0.f; }
    for_int(i, n) { eimag[i] = a[i][i]; }
    {
        auto up_timer = n>1000 ? make_unique<Timer>("__eigenv") : nullptr;
        for_int(iter, INT_MAX) {
            {
                float sum = 0.f; for_int(i, n-1) for_intL(j, i+1, n) { sum += abs(a[i][j]); }
                if (!sum) break;
            }
            for_int(i, n-1) {
                for_intL(j, i+1, n) {
                    float thresh = 1e2f*abs(a[i][j]);
                    if (abs(eimag[i])+thresh==abs(eimag[i]) &&
                        abs(eimag[j])+thresh==abs(eimag[j])) {
                        a[i][j] = 0.f;
                    } else if (abs(a[i][j])>0.f) {
                        float vtan; {
                            float dd = eimag[j]-eimag[i];
                            if (abs(dd)+thresh==abs(dd)) {
                                vtan = a[i][j]/dd;
                            } else {
                                float theta = 0.5f*dd/a[i][j];
                                vtan = 1.f/(abs(theta)+sqrt(1.f+square(theta)));
                                if (theta<0.f) vtan = -vtan;
                            }
                        }
                        float vcos = 1.f/sqrt(1.f+square(vtan)), vsin = vtan*vcos, tau = vsin/(1.f+vcos);
                        eimag[i] -= vtan*a[i][j];
                        eimag[j] += vtan*a[i][j];
                        a[i][j] = 0.f;
                        for_int(k, i)       { rotate(a[k][i],  a[k][j],  tau, vsin); }
                        for_intL(k, i+1, j) { rotate(a[i][k],  a[k][j],  tau, vsin); }
                        for_intL(k, j+1, n) { rotate(a[i][k],  a[j][k],  tau, vsin); }
                        for_int(k, n)       { rotate(mo[i][k], mo[j][k], tau, vsin); }
                    }
                }
            }
            assertx(iter<n*20);
        }
    }
    for_int(i, n) {
        if (eimag[i]<0.f) {
            assertw(eimag[i]>=-1e-5f);
            if (eimag[i]<-1e-5f) { HH_SSTAT(Spca_negv, eimag[i]); }
            eimag[i] = 0.f;
        }
    }
}

void principal_components(CMatrixView<float> mi, MatrixView<float> mo, ArrayView<float> eimag) {
    const int m = mi.ysize(), n = mi.xsize();
    assertx(m>=n && n>=1);
    assertx(mo.ysize()==n && mo.xsize()==n); assertx(eimag.num()==n);
    Matrix<float> a(n, n); {
        // HH_TIMER(_pca_cov);
        auto up_timer = m*n>10000*100 ? make_unique<Timer>("__pca_cov") : nullptr;
        if (1) {                // more cache-coherent
            Matrix<double> t(V(n, n), 0.);
            if (0) {            // unoptimized
                for_int(i, m) for_int(c0, n) for_int(c1, c0+1) { t[c0][c1] += mi[i][c0]*mi[i][c1]; }
            } else {
                for_int(i, m) {
                    const float* mi_i = mi[i].data();
                    for_int(c0, n) {
                        double* t_c0 = t[c0].data();
                        const float mi_i_c0 = mi_i[c0];
                        for_int(c1, c0+1) { t_c0[c1] += mi_i_c0*mi_i[c1]; }
                    }
                }
            }
            for_int(c0, n) for_int(c1, c0+1) { a[c0][c1] = a[c1][c0] = float(t[c0][c1]/m); }
        } else {
            for_int(c0, n) {
                for_int(c1, c0+1) {
                    double sum = 0.; for_int(i, m) { sum += mi[i][c0]*mi[i][c1]; }
                    a[c0][c1] = a[c1][c0] = float(sum/m);
                }
            }
        }
    }
    compute_eigenvectors(a, mo, eimag);
    // Convert variances to standard deviations.
    for_int(i, n) { eimag[i] = sqrt(eimag[i]); }
    // Insertion sort: eigenvalues in descending order.
    for_int(i, n) {
        int imax = i;
        float vmax = eimag[i];
        for_intL(j, i+1, n) { if (eimag[j]>=vmax) { imax = j; vmax = eimag[j]; } }
        if (imax==i) continue;
        std::swap(eimag[i], eimag[imax]);
        swap_ranges(mo[i], mo[imax]);
    }
    // Orient eigenvectors canonically
    Array<float> all1(n, 1.f);
    for_int(i, n) {
        if (dot(mo[i], all1)<0.) mo[i] *= -1.f;
    }
}

// *** incr_principal_components

static float amnesia_factor(int i) {
    float L;
    const int n1 = 20, n2 = 500, m = 1000;
    if (i<n1) {
        L = 0.f;
    } else if (i<n2) {
        L = 2.f*(i-n1)/float(n2-n1);
    } else {
        L = 2.f+float(i-n2)/m;
    }
    return (i-1.f-L)/i;
}

void incr_principal_components(CMatrixView<float> mi, MatrixView<float> mo, ArrayView<float> eimag, int niter) {
    const int m = mi.ysize(), n = mi.xsize(), ne = mo.ysize();
    assertx(m>=1 && n>=1); assertx(ne>=1 && ne<=m && ne<=n); assertx(mo.xsize()==n); assertx(niter>=1);
    assertx(eimag.num()==ne);
    //
    auto up_timer = m*n>10000*100 ? make_unique<Timer>("__pca_inc") : nullptr;
    for_int(i, ne) for_int(c, n) { mo[i][c] = mi[i][c]; }
    Array<float> vnorm(ne); for_int(i, ne) { vnorm[i] = float(assertx(sqrt(mag2(mo[i])))); }
    int count = 2;
    for_int(iter, niter) {
        Array<float> ar(n);     // "data"
        for_int(i, m) {
            for_int(c, n) { ar[c] = mi[i][c]; }
            float w1 = amnesia_factor(count);
            count++;
            for_int(j, ne) {
                float d = (1.f-w1)*float(dot(mo[j], ar))/vnorm[j];
                for_int(c, n) { mo[j][c] = w1*mo[j][c] + d*ar[c]; }
                vnorm[j] = float(assertx(sqrt(mag2(mo[j]))));
                d = float(dot(ar, mo[j]))/square(vnorm[j]);
                for_int(c, n) { ar[c] -= d*mo[j][c]; }
            }
        }
    }
    // Insertion sort: eigenvalues in descending order.
    for_int(i, ne) {
        int imax = i;
        float vmax = vnorm[i];
        for_intL(j, i+1, ne) { if (vnorm[j]>=vmax) { imax = j; vmax = vnorm[j]; } }
        if (imax==i) continue;
        std::swap(vnorm[i], vnorm[imax]);
        swap_ranges(mo[i], mo[imax]);
    }
    // Orthogonalize the approximate eigenvectors
    for_int(i, ne) {
        for_int(j, i) {
            // vi = vi - dot(vi, vj/mag(vj))*vj/mag(vj)  or  = vi - dot(vi, vj)/mag2(vj) * vj
            float d = float(dot(mo[i], mo[j])/mag2(mo[j]));
            for_int(c, n) { mo[i][c] -= d*mo[j][c]; }
        }
    }
    // Normalize them.
    Array<float> all1(n, 1.f);
    for_int(i, ne) {
        float recipnormj = 1.f/vnorm[i];
        if (dot(mo[i], all1)<0.) recipnormj *= -1.f;
        mo[i] *= recipnormj;
    }
    // Convert variances to standard deviations.
    for_int(i, ne) { eimag[i] = sqrt(vnorm[i]); }
}

/// Inspired from Matlab routine:
/// (Weng et al IEEE 2003)
/// 
/// function [V, D, n]=ccipca(X,k,iteration,oldV, access)
/// 
/// %CCIPCA --- Candid Covariance-free Increment Principal Component Analysis
/// %[V,D]=ccipca(X)  ,Batch mode: take input matrix return the eigenvector
/// %matrix
/// %[V,D]=ccipca(X,k) , Batch mode: take input matrix and number of
/// %eigenvector and return the eigenvector and eigenvalue
/// %[V,D]=ccipca(X,k,iteration,oldV,access) , Incremental mode: Take input matirx and
/// %number of eigenvector and number of iteration, and the old eigenvector
/// %matrix, return the eigenvector and eigenvalue matrix
/// %
/// %[V,D]=ccipca(...) return both the eigenvector and eigenvalue matrix
/// %V=ccipca(...) return only the eigenvector matrix
/// %Algorithm
/// 
/// %ARGUMENTS:
/// %INPUT
/// %X --- Sample matrix, samples are column vectors. Note the samples must be
/// %centered (subtracted by mean)
/// %k --- number of eigenvectors that will be computed
/// %iteration --- number of times the data are used
/// %oldV --- old eigen vector matrix, column wise
/// %access --- the number of updatings of the old eigenvector matrix
/// %OUTPUT
/// %V --- Eigenvector matrix, column-wise
/// %D --- Diagonal matrix of eigenvalue
/// %n --- Updating occurance
/// 
///   [datadim, samplenum] = size(X);
/// 
///   %samplemean = mean(X,2);
///   %scatter = X-samplemean*ones(1, samplenum); %subtract the sample set by its mean
///   vectornum = datadim;
///   repeating = 1;
///   n = 2; % the number of times the eigenvector matrix get updated. Magic number to prevent div by 0 error
/// 
///   if nargin == 1
///     % batch mode, init the eigenvector matrix with samples
///     if datadim>samplenum
///       error('No. of samples is less than the dimension. You have to choose how many eigenvectors to compute. ');
///     end
///     V = X(:,1:datadim);
///   elseif nargin == 2
///     % number of eigenvector given
///     if k > datadim
///       k = datadim;
///     end
///     vectornum = k;
///     V = X(:,1:vectornum);
///   elseif nargin == 3
///     % number of eigenvector given, number of iteration given
///     if k > datadim
///       k = datadim;
///     end
///     vectornum = k;
///     V = X(:,1:vectornum);
///     repeating = iteration;
///   elseif nargin == 4
///     % if given oldV the the argument k will not take effect.
///     if datadim~=size(oldV,1)
///       error('The dimensionality of sample data and eigenvector are not match. Program ends.');
///     end
///     vectornum = size(oldV,2);
///     V = oldV;
///     repeating = iteration;
///     n = access;
///   end
///   Vnorm = sqrt(sum(V.^2));
///   for iter = 1:repeating
///     for  i = 1:samplenum
///       residue = X(:, i);  % get the image
///       [w1, w2] = amnesic(n);
///       n = n+1;
///       for j= 1:vectornum
///         V(: , j) = w1 * V(:,j) + w2 * V(:,j)' * residue * residue / Vnorm(j);
///         Vnorm(j) = norm(V(:,j)); % updata the norm of eigen vecor
///         normedV = V(:,j)/Vnorm(j);
///         residue = residue - residue' * normedV * normedV;
///       end
///     end
///   end
///   D = sqrt(sum(V.^2)); %length of the updated eigen vector, aka eigen value
///   [Y, I] = sort(-D);
///   V = V(:,I);
///   V = normc(V); %normalize V
///   if nargout>=2
///     D = D(I);
///     D = diag(D);
///   end
/// 
/// return
///
///
/// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/// function [w1, w2]=amnesic(i)
/// %AMNESIC --- Calculate the amnesic weight
/// %
/// %INPUT
/// %i --- accessing time
/// %OUTPUT
/// %w1, w2 ---two amnesic weights
/// 
///   n1 = 20;
///   n2 = 500;
///   m = 1000;
///   if i < n1
///     L = 0;
///   elseif i >= n1 & i < n2
///     L = 2*(i-n1)/(n2-n1);
///   else
///     L = 2+(i-n2)/m;
///   end
///   w1 = (i-1-L)/i;
///   w2 = (1+L)/i;
/// 
/// return
/// 
/// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

} // namespace hh
