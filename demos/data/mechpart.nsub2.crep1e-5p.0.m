# Created using:
#  Subdivfit -mf - -fi mechpart.4102.pts -record mechpart.nsub2.crep1e-5p.rec
#   -crep 1e-5 -csharp .2e-5 -reconstruct
# Initial mesh:
# Genus: c=1 b=0  v=163 f=334 e=501  genus=3  sharpe=136 cuspv=0
# 4102 points read
# Starting reconstruction sequence
#  crep=1e-05, csharp=2e-06
#  internal xform: F 1  1 0 0  0 1 0  0 0 1  0 0 0  0
# 
# fgfit_before: v=163 nse=136/501  edis=0.00655278 etot=0.00845478
# fgfit_after : v=163 nse=136/501  edis=0.000165907 etot=0.00206791
#  (_fgfit:                446.33)
# 
# Stoc, crep=1e-05 csharp=2e-06 wcrep=1e-05 wcsharp=2e-06
# stoc_before: v=163 nse=136/501  edis=0.000165905 etot=0.0020679
# it 634, last search: 55 wasted attempts
# New mesh:
# Genus: c=1 b=0  v=118 f=244 e=366  genus=3  sharpe=117 cuspv=0
# Summary of attempts and results:
#                           ecol      espl      eswa      esha
#       total_attempts       634         0         0         0
#              success        45         0         0         0
#      positive_energy       166         0         0         0
#         bad_dihedral        61         0         0         0
#            bad_sharp       354         0         0         0
#         illegal_move         8         0         0         0
#  last cedis=0.000252552 cetot=0.00166655
# stoc_after : v=118 nse=117/366  edis=0.000252554 etot=0.00166655
#  (_stoc:                1943.25)
# 
# fgfit_before: v=118 nse=117/366  edis=0.000252551 etot=0.00166655
# fgfit_after : v=118 nse=117/366  edis=0.000191469 etot=0.00160547
#  (_fgfit:                128.00)
# 
# Stoc, crep=1e-05 csharp=2e-06 wcrep=1e-05 wcsharp=2e-06
# stoc_before: v=118 nse=117/366  edis=0.00019147 etot=0.00160547
# it 368, last search: 361 wasted attempts
# New mesh:
# Genus: c=1 b=0  v=118 f=244 e=366  genus=3  sharpe=116 cuspv=0
# Summary of attempts and results:
#                           ecol      espl      eswa      esha
#       total_attempts         0         0         0       368
#              success         0         0         0         1
#      positive_energy         0         0         0       152
#         bad_dihedral         0         0         0         0
#            bad_sharp         0         0         0       215
#         illegal_move         0         0         0         0
#  last cedis=0.000188079 cetot=0.00160008
# stoc_after : v=118 nse=116/366  edis=0.000188082 etot=0.00160008
#  (_stoc:                 842.88)
# 
# fgfit_before: v=118 nse=116/366  edis=0.000188082 etot=0.00160008
# fgfit_after : v=118 nse=116/366  edis=0.000172819 etot=0.00158482
#  (_fgfit:                137.00)
# 
# Stoc, crep=1e-05 csharp=2e-06 wcrep=1e-05 wcsharp=2e-06
# stoc_before: v=118 nse=116/366  edis=0.000172819 etot=0.00158482
# it 407, last search: 9 wasted attempts
# New mesh:
# Genus: c=1 b=0  v=115 f=238 e=357  genus=3  sharpe=113 cuspv=0
# Summary of attempts and results:
#                           ecol      espl      eswa      esha
#       total_attempts       407         0       404       404
#              success         3         0        10         0
#      positive_energy       140         0       211       169
#         bad_dihedral       261         0       180         0
#            bad_sharp         0         0         0       235
#         illegal_move         3         0         3         0
#  last cedis=0.000179567 cetot=0.00155557
# stoc_after : v=115 nse=113/357  edis=0.000179743 etot=0.00155574
#  (_stoc:                3603.72)
# 
# fgfit_before: v=115 nse=113/357  edis=0.000179742 etot=0.00155574
# fgfit_after : v=115 nse=113/357  edis=0.000157878 etot=0.00153388
#  (_fgfit:                125.77)
#  (Subdivfit:            7228.12)
# Summary of timers (cpu=mips host=marlin.cs.washington.edu):
#  ___submesh:         (1229  )    0.67:4.98     av=     1.89   sum=  2323.00
#  ____gmakeSpatial:   (10    )    6.25:7.82     av=     6.60   sum=    66.03
#  ____gspatialproject:(10    )    2.12:2.52     av=     2.25   sum=    22.52
#  ___gallproject:     (10    )    8.42:10.37    av=     8.92   sum=    89.23
#  ___computegrad:     (267   )    0.55:0.60     av=     0.56   sum=   149.90
#  ___gneighproject:   (468   )    1.17:1.32     av=     1.21   sum=   568.55
#  __fgfit_iter:       (70    )    9.03:15.25    av=    11.16   sum=   781.23
#  _fgfit:             (4     )  125.77:446.33   av=   209.28   sum=   837.10
#  _initial_fit:       (3     )   11.68:14.18    av=    12.53   sum=    37.58
#  ____lmakespatial:   (1219  )    0.00:0.83     av=     0.17   sum=   202.82
#  ____lspatialproject:(1219  )    0.02:1.42     av=     0.12   sum=   150.60
#  ___lallproject:     (1219  )    0.02:2.25     av=     0.29   sum=   355.27
#  ____lneighproject:  (4505  )    0.12:1.12     av=     0.42   sum=  1912.57
#  ____lcombinations:  (4505  )    0.02:0.33     av=     0.08   sum=   354.88
#  _____qrdLLS:        (4505  )    0.00:1.43     av=     0.10   sum=   453.97
#  ____lsolve:         (4505  )    0.00:1.58     av=     0.13   sum=   598.28
#  ___loptimize:       (4505  )    0.17:3.17     av=     0.67   sum=  3017.85
#  __tryecol:          (676   )    2.00:16.28    av=     5.80   sum=  3920.37
#  __lattempt:         (1409  )    0.00:24.35    av=     4.48   sum=  6316.67
#  _stoc:              (3     )  842.88:3603.72  av=  2129.95   sum=  6389.85
#  __tryesha:          (322   )    2.67:13.03    av=     5.32   sum=  1713.28
#  __tryeswa:          (221   )    1.48:8.68     av=     2.99   sum=   660.28
#  Subdivfit:          (1     )        :         av=  7228.12   sum=  7228.12
# Summary of statistics:
# Sospobcells:(149120 )          1:13          av=3.20249     sd=1.48407
# Sssncellsv: (168961 )          1:216         av=5.04976     sd=13.2176
# Sssnelemsv: (168961 )          1:138         av=13.824      sd=11.9139
# Sospcelln:  (68374  )          1:39          av=6.98445     sd=4.96235
# Sprojquick: (8300407)          0:1           av=0.46609     sd=0.498849
# Sprojnei:   (4431667)          0:5           av=0.0529374   sd=0.235241
# Sprojf:     (4431667)          4:54          av=11.4883     sd=3.07294
# Sprojunexp: (224681 )          0:1           av=0.00379204  sd=0.0614628
# Sneval:     (70     )          6:9           av=6.68571     sd=0.692462
# Stmin:      (70     )  0.0198115:0.0906504   av=0.0550072   sd=0.0141922
# Ssetfrem:   (1219   )        464:1968        av=1219.47     sd=235.216
# Ssetvrem:   (1219   )        239:996         av=606.208     sd=115.731
# Secolpts:   (676    )        465:3417        av=1695.98     sd=548.964
# Secolmf:    (676    )         40:213         av=104.441     sd=32.9176
# Secolmv:    (676    )          5:27          av=10.2751     sd=3.6051
# Secolsmv:   (676    )        349:1786        av=903.32      sd=273.81
# Soptnit:    (905    )          2:10          av=3.78011     sd=2.05059
# Soptnig:    (67     )         11:12          av=11.1493     sd=0.359028
# Sechange:   (59     )-2.38792e-05:-7.17789e-07 av=-7.38976e-06 sd=5.26772e-06
# Seshapts:   (322    )        727:3417        av=1705.47     sd=541.629
# Seshamf:    (322    )         47:199         av=103.919     sd=33.8957
# Seshamv:    (322    )          7:28          av=11.0342     sd=4.12674
# Seshasmv:   (322    )        411:1662        av=898.652     sd=280.334
# Seswapts:   (221    )        450:2037        av=999.909     sd=379.339
# Seswamf:    (221    )         28:122         av=60.4796     sd=20.9984
# Seswamv:    (221    )          4:4           av=4           sd=0
# Seswamvcvih:(221    )        249:1078        av=534.955     sd=184.36
Vertex 1  0.453022 0.454306 0.433115
Vertex 4  0.366831 0.433569 0.416937
Vertex 7  0.3987 0.401222 0.470254
Vertex 8  0.400287 0.366555 0.474328
Vertex 11  0.366669 0.366663 0.366662
Vertex 14  0.366584 0.401681 0.489894
Vertex 28  0.453016 0.366807 0.399771
Vertex 36  0.502821 0.498464 0.399848
Vertex 39  0.448699 0.366684 0.366664
Vertex 45  0.411991 0.366729 0.560513
Vertex 48  0.366505 0.504019 0.402068
Vertex 50  0.366705 0.460613 0.59864
Vertex 51  0.366669 0.409256 0.539348
Vertex 53  0.366656 0.633327 0.366651
Vertex 54  0.397337 0.478411 0.366674
Vertex 56  0.417971 0.438007 0.366671
Vertex 65  0.408491 0.407335 0.551652
Vertex 69  0.453066 0.406816 0.366672
Vertex 85  0.366536 0.581111 0.445097
Vertex 87  0.366876 0.552133 0.416306
Vertex 92  0.421626 0.585988 0.446919
Vertex 93  0.40168 0.541092 0.366669
Vertex 104  0.553205 0.366593 0.408258
Vertex 111  0.493077 0.487965 0.596367
Vertex 120  0.395464 0.484778 0.100004
Vertex 123  0.633331 0.366679 0.366684
Vertex 130  0.58748 0.40927 0.446337
Vertex 133  0.461357 0.533012 0.422972
Vertex 137  0.480005 0.366583 0.601114
Vertex 138  0.524896 0.366652 0.633305
Vertex 141  0.551541 0.410725 0.366671
Vertex 143  0.505215 0.395897 0.366664
Vertex 164  0.63327 0.456571 0.406647
Vertex 171  0.366604 0.545493 0.593445
Vertex 174  0.398227 0.470619 0.633369
Vertex 176  0.366688 0.366665 0.633283
Vertex 194  0.472514 0.633332 0.400522
Vertex 197  0.509458 0.491759 0.59093
Vertex 208  0.398075 0.478083 0.899997
Vertex 210  0.434853 0.416719 0.633453
Vertex 219  0.459193 0.403741 0.100002
Vertex 232  0.633404 0.401616 0.466887
Vertex 244  0.584874 0.440772 0.36667
Vertex 251  0.366643 0.633342 0.633331
Vertex 258  0.483223 0.398571 0.633289
Vertex 261  0.436137 0.582212 0.366659
Vertex 269  0.582563 0.366735 0.460002
Vertex 277  0.551324 0.553046 0.429812
Vertex 280  0.553787 0.633159 0.411621
Vertex 286  0.477351 0.514345 0.592586
Vertex 296  0.398424 0.526093 0.633311
Vertex 316  0.602211 0.401419 0.525112
Vertex 326  0.397749 0.597614 0.498702
Vertex 327  0.417008 0.586409 0.550556
Vertex 328  0.36685 0.595783 0.536694
Vertex 329  0.4103 0.633334 0.535115
Vertex 340  0.526663 0.398228 0.633368
Vertex 341  0.535014 0.401509 0.900001
Vertex 343  0.483213 0.60139 0.366692
Vertex 360  0.602608 0.484142 0.366671
Vertex 367  0.512056 0.396599 0.0999971
Vertex 380  0.424014 0.425971 0.900018
Vertex 383  0.451711 0.596787 0.0999962
Vertex 386  0.598621 0.534148 0.366671
Vertex 388  0.633337 0.633369 0.366676
Vertex 391  0.566649 0.489512 0.597864
Vertex 394  0.533928 0.531577 0.59128
Vertex 400  0.430751 0.633353 0.5713
Vertex 418  0.479843 0.396225 0.899998
Vertex 420  0.586766 0.433317 0.633282
Vertex 425  0.567671 0.579524 0.366677
Vertex 426  0.524226 0.604064 0.0999938
Vertex 433  0.602155 0.36659 0.502084
Vertex 435  0.633274 0.557463 0.418895
Vertex 439  0.633379 0.515777 0.405345
Vertex 444  0.602595 0.484691 0.0999974
Vertex 449  0.599679 0.633518 0.46745
Vertex 453  0.537481 0.453655 0.57863
Vertex 462  0.424681 0.573186 0.633326
Vertex 480  0.598813 0.530367 0.100001
Vertex 481  0.417784 0.435844 0.0999878
Vertex 486  0.633332 0.366659 0.633355
Vertex 502  0.592247 0.593397 0.469067
Vertex 512  0.633343 0.428329 0.564709
Vertex 514  0.633302 0.413601 0.546204
Vertex 517  0.554101 0.412767 0.0999984
Vertex 519  0.490154 0.633265 0.602092
Vertex 538  0.527538 0.599139 0.366651
Vertex 544  0.585109 0.441355 0.0999993
Vertex 551  0.633332 0.472708 0.597682
Vertex 560  0.472773 0.600835 0.633396
Vertex 562  0.528439 0.63334 0.633348
Vertex 573  0.574413 0.573524 0.100003
Vertex 631  0.580764 0.431277 0.900002
Vertex 643  0.633341 0.633329 0.633342
Vertex 645  0.633365 0.545141 0.589699
Vertex 653  0.517641 0.600128 0.633203
Vertex 659  0.474726 0.602926 0.899979
Vertex 662  0.399391 0.527555 0.899989
Vertex 663  0.591718 0.55562 0.899996
Vertex 665  0.587722 0.555384 0.633333
Vertex 666  0.606699 0.508321 0.633349
Vertex 681  0.633287 0.586626 0.550959
Vertex 683  0.597935 0.596826 0.532889
Vertex 691  0.588599 0.633248 0.549296
Vertex 692  0.555506 0.633408 0.5813
Vertex 696  0.553952 0.587305 0.633316
Vertex 760  0.425222 0.573717 0.900012
Vertex 784  0.539091 0.599044 0.900042
Vertex 808  0.60679 0.492667 0.899998
Vertex 992  0.409064 0.633326 0.44463
Vertex 1103  0.567176 0.366754 0.585319
Vertex 1181  0.404527 0.549136 0.0999959
Vertex 1316  0.633345 0.585595 0.451793
Vertex 1392  0.633376 0.595894 0.497329
Face 62  111 137 45
Face 116  244 123 141
Face 157  232 130 316
Face 163  329 327 326
Face 166  251 328 171
Face 171  340 341 258
Face 204  400 286 327
Face 226  449 388 280
Face 253  449 277 502
Face 324  208 174 380
Face 427  663 696 784
Face 468  1 4 7
Face 483  4 14 7
Face 516  14 51 7
Face 592  36 1 28
Face 595  4 1 36
Face 650  48 4 36
Face 683  28 104 36
Face 712  87 53 48
Face 747  87 48 36
Face 797  133 87 36
Face 801  111 45 65
Face 866  87 133 92
Face 910  210 176 138
Face 933  54 93 120
Face 954  164 36 130
Face 973  111 50 171
Face 996  210 174 176
Face 999  138 258 210
Face 1041  194 133 36
Face 1068  137 111 197
Face 1138  269 104 123
Face 1141  104 269 130
Face 1169  133 992 92
Face 1170  194 992 133
Face 1186  328 85 326
Face 1188  992 326 92
Face 1284  164 130 232
Face 1307  111 171 286
Face 1308  197 1103 137
Face 1309  453 1103 197
Face 1335  69 219 143
Face 1417  439 36 164
Face 1468  326 992 329
Face 1491  251 296 462
Face 1576  123 244 360
Face 1614  453 197 391
Face 1646  143 367 141
Face 1690  343 426 383
Face 1699  383 93 261
Face 1702  1181 120 93
Face 1703  383 1181 93
Face 1708  120 56 54
Face 1762  36 277 280
Face 1787  329 251 400
Face 1789  327 329 400
Face 1844  538 426 343
Face 1862  481 56 120
Face 1948  367 143 219
Face 2081  453 551 512
Face 2086  286 400 519
Face 2097  517 141 367
Face 2149  425 388 386
Face 2183  514 512 486
Face 2202  277 449 280
Face 2216  551 453 391
Face 2220  286 519 394
Face 2234  517 544 244
Face 2294  425 386 573
Face 2358  645 551 391
Face 2425  538 425 573
Face 2443  1316 1392 502
Face 2483  519 562 692
Face 2549  538 573 426
Face 2572  683 502 1392
Face 2575  1392 681 683
Face 2584  502 683 449
Face 2627  659 560 462
Face 2632  462 760 659
Face 2639  662 760 296
Face 2696  449 691 388
Face 2703  645 394 681
Face 2792  691 394 692
Face 2793  691 683 394
Face 2794  394 683 681
Face 2977  696 663 665
Face 3032  420 808 631
Face 3130  663 808 665
Face 3153  394 645 391
Face 3159  85 92 326
Face 3165  210 380 174
Face 3170  691 449 683
Face 3181  328 251 53
Face 3192  1392 1316 388
Face 3218  433 486 1103
Face 3239  696 653 784
Face 3240  659 784 653
Face 3255  244 141 517
Face 3271  453 512 316
Face 3272  514 316 512
Face 3286  665 666 643
Face 3301  36 104 130
Face 3313  232 316 514
Face 3349  327 328 326
Face 3390  51 65 7
Face 3397  280 388 194
Face 3404  280 194 36
Face 3405  643 645 681
Face 3410  328 53 85
Face 3415  269 433 130
Face 3416  316 130 433
Face 3430  92 85 87
Face 3467  643 388 691
Face 3476  519 692 394
Face 3478  573 386 480
Face 3483  328 327 171
Face 3484  286 171 327
Face 3485  380 418 208
Face 3498  992 194 53
Face 3521  251 329 53
Face 3522  992 53 329
Face 3524  45 11 8
Face 3532  551 486 512
Face 3535  258 418 210
Face 3536  380 210 418
Face 3544  341 418 258
Face 3547  435 1316 277
Face 3548  502 277 1316
Face 3569  11 45 176
Face 3571  251 562 400
Face 3572  519 400 562
Face 3577  435 388 1316
Face 3582  123 104 39
Face 3584  653 560 659
Face 3586  462 296 760
Face 3591  65 50 111
Face 3595  50 65 51
Face 3597  277 36 435
Face 3598  439 435 36
Face 3623  251 176 296
Face 3624  174 296 176
Face 3631  666 665 808
Face 3638  435 439 388
Face 3641  261 53 343
Face 3657  643 691 562
Face 3658  692 562 691
Face 3699  388 53 194
Face 3704  50 51 176
Face 3712  164 232 123
Face 3730  261 93 53
Face 3731  666 808 420
Face 3737  208 662 174
Face 3738  296 174 662
Face 3741  50 176 171
Face 3742  251 171 176
Face 3745  261 343 383
Face 3752  56 11 54
Face 3755  4 11 14
Face 3777  269 123 433
Face 3778  486 433 123
Face 3816  538 388 425
Face 3830  85 53 87
Face 3839  164 123 439
Face 3840  388 439 123
Face 3881  11 56 39
Face 3882  69 39 56
Face 3911  4 48 11
Face 3912  53 11 48
Face 3913  53 93 11
Face 3914  54 11 93
Face 3921  643 681 388
Face 3922  1392 388 681
Face 3925  56 481 69
Face 3926  219 69 481
Face 3937  433 1103 316
Face 3938  453 316 1103
Face 3941  11 176 14
Face 3942  51 14 176
Face 3985  343 53 538
Face 3986  388 538 53
Face 3994  341 631 418
Face 4021  560 653 562
Face 4039  232 514 123
Face 4040  486 123 514
Face 4042  418 631 208
Face 4055  251 462 562
Face 4056  560 562 462
Face 4100  808 663 631
Face 4125  386 360 480
Face 4126  444 480 360
Face 4143  420 486 666
Face 4144  643 666 486
Face 4176  784 659 663
Face 4221  486 551 643
Face 4222  645 643 551
Face 4241  360 244 444
Face 4242  544 444 244
Face 4271  69 143 39
Face 4283  123 39 141
Face 4284  143 141 39
Face 4285  123 360 388
Face 4286  386 388 360
Face 4327  631 663 208
Face 4363  137 138 45
Face 4364  176 45 138
Face 4394  367 219 517
Face 4398  481 120 219
Face 4422  653 696 562
Face 4434  219 120 517
Face 4435  643 562 665
Face 4436  696 665 562
Face 4437  662 208 760
Face 4451  208 663 760
Face 4452  659 760 663
Face 4462  517 120 544
Face 4530  1181 383 120
Face 4570  573 480 426
Face 4579  426 120 383
Face 4582  426 480 120
Face 4607  480 444 120
Face 4613  444 544 120
Face 4659  137 1103 138
Face 4660  486 138 1103
Face 4663  1 7 28
Face 4664  8 28 7
Face 4666  391 197 394
Face 4667  8 11 28
Face 4669  197 111 394
Face 4670  286 394 111
Face 4671  28 11 104
Face 4672  39 104 11
Face 4673  7 65 8
Face 4674  45 8 65
Face 4676  420 340 486
Face 4677  138 486 258
Face 4678  340 258 486
Face 4679  341 340 631
Face 4680  420 631 340
Edge 340 420 {sharp}
Edge 1 7 {sharp}
Edge 341 631 {sharp}
Edge 341 418 {sharp}
Edge 681 1392 {sharp}
Edge 343 538 {sharp}
Edge 4 48 {sharp}
Edge 4 14 {sharp}
Edge 232 514 {sharp}
Edge 7 65 {sharp}
Edge 120 481 {sharp}
Edge 120 1181 {sharp}
Edge 8 28 {sharp}
Edge 8 45 {sharp}
Edge 462 560 {sharp}
Edge 123 388 {sharp}
Edge 123 486 {sharp}
Edge 11 39 {sharp}
Edge 11 176 {sharp}
Edge 11 53 {sharp}
Edge 691 692 {sharp}
Edge 14 51 {sharp}
Edge 130 316 {sharp}
Edge 244 360 {sharp}
Edge 360 386 {sharp}
Edge 137 1103 {sharp}
Edge 251 562 {sharp}
Edge 138 486 {sharp}
Edge 138 176 {sharp}
Edge 480 573 {sharp}
Edge 367 517 {sharp}
Edge 28 104 {sharp}
Edge 141 143 {sharp}
Edge 141 244 {sharp}
Edge 258 340 {sharp}
Edge 486 643 {sharp}
Edge 261 343 {sharp}
Edge 36 130 {sharp}
Edge 39 123 {sharp}
Edge 380 418 {sharp}
Edge 269 433 {sharp}
Edge 383 1181 {sharp}
Edge 383 426 {sharp}
Edge 45 137 {sharp}
Edge 386 425 {sharp}
Edge 48 87 {sharp}
Edge 388 643 {sharp}
Edge 502 683 {sharp}
Edge 50 171 {sharp}
Edge 50 51 {sharp}
Edge 164 232 {sharp}
Edge 164 439 {sharp}
Edge 277 502 {sharp}
Edge 53 388 {sharp}
Edge 53 251 {sharp}
Edge 54 93 {sharp}
Edge 54 56 {sharp}
Edge 280 449 {sharp}
Edge 394 683 {sharp}
Edge 56 69 {sharp}
Edge 171 328 {sharp}
Edge 512 551 {sharp}
Edge 512 514 {sharp}
Edge 286 327 {sharp}
Edge 400 519 {sharp}
Edge 174 296 {sharp}
Edge 174 210 {sharp}
Edge 176 251 {sharp}
Edge 517 544 {sharp}
Edge 65 111 {sharp}
Edge 631 808 {sharp}
Edge 519 692 {sharp}
Edge 69 143 {sharp}
Edge 296 462 {sharp}
Edge 1316 1392 {sharp}
Edge 645 681 {sharp}
Edge 420 666 {sharp}
Edge 194 992 {sharp}
Edge 194 280 {sharp}
Edge 85 87 {sharp}
Edge 85 328 {sharp}
Edge 425 538 {sharp}
Edge 426 573 {sharp}
Edge 653 696 {sharp}
Edge 316 453 {sharp}
Edge 92 133 {sharp}
Edge 92 326 {sharp}
Edge 93 261 {sharp}
Edge 659 784 {sharp}
Edge 659 760 {sharp}
Edge 433 1103 {sharp}
Edge 208 662 {sharp}
Edge 208 380 {sharp}
Edge 435 439 {sharp}
Edge 435 1316 {sharp}
Edge 662 760 {sharp}
Edge 210 258 {sharp}
Edge 663 808 {sharp}
Edge 663 784 {sharp}
Edge 551 645 {sharp}
Edge 665 666 {sharp}
Edge 665 696 {sharp}
Edge 326 327 {sharp}
Edge 329 992 {sharp}
Edge 329 400 {sharp}
Edge 104 269 {sharp}
Edge 444 544 {sharp}
Edge 444 480 {sharp}
Edge 219 481 {sharp}
Edge 219 367 {sharp}
Edge 560 653 {sharp}
Edge 562 643 {sharp}
Edge 449 691 {sharp}
