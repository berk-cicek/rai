#include "NLP_Sampler.h"

#include <Core/util.h>
#include <math.h>

/*
 * void NLP_Sampler::step_ball(uint steps){
#if 0
  arr g = A*x - b;
  arr g2 = (g%g);
  g2 += 1e-6;
  arr g2_1 = ones(g.N);
  g2_1 /= g2;
  arr At = ~A;
  arr H = At*(g2%A);
  arr C;
  lapack_cholesky(C, H);
  arr Cinv = inverse(C);
  //    Cinv.setId();
#endif
  for(uint t=0,s=0;s<steps;t++){
    //      arr y = x + Cinv * (1.*randn(x.N));
    arr y = x + (.2*randn(x.N));
    samples++;
    //      arr y = nlp.getInitializationSample();
    bool inBound = boundCheck(y, nlp.bounds_lo, nlp.bounds_up, 1e-6, false);
    if(inBound && max(g)<=0.){ //accept
      x = y;
      s++;
    }
  }
}
*/

void NLP_Walker::set_alpha_bar(double alpha_bar){
  if(alpha_bar==1.){
    a = 1.;
    sig = 0.;
  }else{
    a = ::sqrt(alpha_bar);
    sig = ::sqrt(1.-alpha_bar);
  }
  if(!opt.useCentering){
    a = 1.;
  }
}

bool NLP_Walker::step(){
  ensure_eval();

  get_delta();
  bool good = true;
  if(!ev.Ph.N || trace(ev.Ph)>1e-6){
    good = step_hit_and_run(opt.maxStep);
  }

  //  bool good = step_delta();
  good = step_slack();

  return good;
}

bool NLP_Walker::step_hit_and_run(double maxStep){
  ensure_eval();
  Eval ev0 = ev;

  arr dir = get_rnd_direction();

  double beta_mean, beta_sdv;
  get_beta_mean(beta_mean, beta_sdv, dir, x);

  boundClip(x, nlp.bounds_lo, nlp.bounds_up);

  LineSampler LS(2.*maxStep);
  LS.clip_beta(nlp.bounds_lo - x, -dir); //cut with lower bound
  LS.clip_beta(x - nlp.bounds_up, dir); //cut with upper bound
  LS.add_constraints(a*ev.g + ev.Jg*(x-a*ev.x), ev.Jg*dir, sig);
  for(uint i=0;i<10;i++){ //``line search''
    double beta = NAN;
    if(!sig){
      //cut with constraints
      LS.clip_beta(ev.g + ev.Jg*(x-ev.x), ev.Jg*dir);

      if(LS.beta_lo >= LS.beta_up) break; //failure

      if(beta_sdv<0.){
        beta = LS.sample_beta_uniform();
      }else{ //Gaussian beta
        bool good=false;
        for(uint k=0;k<10;k++){
          beta = beta_mean + beta_sdv*rnd.gauss();
          if(beta>=LS.beta_lo && beta<=LS.beta_up){ good=true; break; }
        }
        if(!good){
          beta = LS.sample_beta_uniform();
          //          LOG(0) <<evals <<"uniform";
        }
      }
    }else{
      if(LS.beta_up>LS.beta_lo){
        beta = LS.sample_beta();
        //    cout <<"beta: " <<beta <<" p(beta): " <<LS.p_beta <<endl;
        if(LS.p_beta<1e-10) beta=NAN;
      }
      if(isnan(beta)) break;
    }

    x += beta*dir;
    samples++;
    ensure_eval();

    if(!sig){
      if((!ev.g.N || max(ev.gpos) <= max(ev0.gpos)) //ineq constraints are good
         && sum(ev.s) <= sum(ev0.s) + opt.eps){ //total slack didn't increase too much
        return true;
      }
    }else{
      if(sum(ev.s) <= sum(ev0.s) + sig + opt.eps){ //total slack didn't increase too much
        if(!ev.g.N) return true;
        LS.add_constraints(a*ev.g + ev.Jg*(x-a*ev.x), ev.Jg*dir, sig);
        double p_beta = LS.eval_beta(beta);
        //      cout <<"beta: " <<beta <<" p(beta): " <<LS.p_beta <<" p(beta) " <<p_beta <<endl;
        if(p_beta >= 1e-3*LS.p_beta){
          return true;
        }
      }
    }
  }

  ev = ev0;
  x = ev.x;
  return false; //line search in 10 steps failed
}

bool NLP_Walker::step_slack(){
  ensure_eval();

#if 0
  arr s0 = sig * randn(ev.s.N);
  arr delta_s = alpha * (s0 - ev.s);
  arr delta = pseudoInverse(ev.Js) * delta_s;

  double l = length(delta);
  if(l>maxStep) delta *= maxStep/l;

  x += delta;
  boundClip(x, nlp.bounds_lo, nlp.bounds_up);
#else
//  if(!ev.h.N) return true;
  arr Jinv = pseudoInverse(ev.Js);
  arr delta = -opt.alpha *( Jinv * ev.s );
  arr Ph = Jinv * ev.Js;

  double l = length(delta);
  if(l>opt.maxStep) delta *= opt.maxStep/l;

  x += delta;
  x *= a;
  if(sig){
    x += Ph * (sig * randn(x.N));
  }
  if(!sig){
    boundClip(x, nlp.bounds_lo, nlp.bounds_up);
  }

#endif

  ensure_eval();
  return true;
}

arr NLP_Walker::get_delta(){
  arr delta;
  if(ev.s.N && absMax(ev.s)>1e-9){
    //Gauss-Newton direction
    arr H = 2. * ~ev.Js * ev.Js;
    arr grad = 2. * ~ev.Js * ev.s;
    arr Hinv = pseudoInverse(H);
    delta = -(Hinv * grad);
  }

  if(ev.h.N){ //equality constraints
    //Gauss-Newton direction
    arr H = 2. * ~ev.Jh * ev.Jh;
    arr Hinv = pseudoInverse(H);
    ev.Ph = eye(x.N) - Hinv * H; //tangent projection
  }else{
    ev.Ph.clear();
  }

  return delta;
}

void NLP_Walker::get_beta_mean(double& beta_mean, double& beta_sdv, const arr& dir, const arr& xbar) {
  beta_mean = 0.;
  beta_sdv = -1.;
  if(ev.r.N){
    arr r_bar = ev.r + ev.Jr*(xbar - ev.x);
    arr Jr_d = ev.Jr * dir;
    double Jr_d_2 = sumOfSqr(Jr_d);
    if(Jr_d_2>1e-6){
      beta_mean = - scalarProduct(r_bar, Jr_d) / Jr_d_2;
      beta_sdv = sqrt(0.5/Jr_d_2);
    }
  }
}

arr NLP_Walker::get_rnd_direction(){
  arr dir = randn(x.N);
  if(ev.Ph.N) dir = ev.Ph * dir;
  dir /= length(dir);
  return dir;
}

void NLP_Walker::Eval::eval(const arr& _x, NLP_Walker& walker){
  if(x.N && maxDiff(_x, x)<1e-10){
    return; //already evaluated
  }
  x = _x;
  walker.evals++;

  phi, J;
  walker.nlp.evaluate(phi, J, _x);
  if(rai::isSparse(J)) J = J.sparse().unsparse();

  {//grab ineqs
    uintA ineqIdx;
    for(uint i=0;i<walker.nlp.featureTypes.N;i++) if(walker.nlp.featureTypes(i)==OT_ineq) ineqIdx.append(i);
    g = phi.sub(ineqIdx);
    Jg = J.sub(ineqIdx);
  }

  {//grab eqs
    uintA eqIdx;
    for(uint i=0;i<walker.nlp.featureTypes.N;i++) if(walker.nlp.featureTypes(i)==OT_eq) eqIdx.append(i);
    h = phi.sub(eqIdx);
    Jh = J.sub(eqIdx);
  }

  {//define slack
    s = g; Js = Jg;
    for(uint i=0;i<s.N;i++) if(s(i)<0.){ s(i)=0.; Js[i]=0.; } //ReLu for g
    gpos = s;

    s.append(h); Js.append(Jh);
    for(uint i=g.N;i<s.N;i++) if(s(i)<0.){ s(i)*=-1.; Js[i]*=-1.; } //make positive

    err = sum(s);
  }

  {//grab sos
    uintA sosIdx;
    for(uint i=0;i<walker.nlp.featureTypes.N;i++) if(walker.nlp.featureTypes(i)==OT_sos) sosIdx.append(i);
    r = phi.sub(sosIdx);
    Jr = J.sub(sosIdx);
  }
}

//===========================================================================

arr sample_direct(NLP& nlp, uint K, int verbose, double alpha_bar){
  NLP_Walker walk(nlp, alpha_bar);

//  walk.initialize(nlp.getInitializationSample());
  walk.initialize(nlp.getUniformSample());

  arr data;

  for(;data.d0<K;){
    bool good = walk.step();

    if(good){
      data.append(walk.x);
      data.reshape(-1, walk.x.N);
      if(!(data.d0%10)) cout <<'.' <<std::flush;
    }

    if(verbose>1 || (good && verbose>0)){
      nlp.report(cout, 2+verbose, STRING("sample_direct it: " <<data.d0 <<" good: " <<good));
    }
  }
  cout <<"\nsteps/sample: " <<double(walk.samples)/K <<" evals/sample: " <<double(walk.evals)/K <<endl;

  data.reshape(-1, nlp.getDimension());
  return data;
}

//===========================================================================

arr sample_restarts(NLP& nlp, uint K, int verbose, double alpha_bar){
  NLP_Walker walk(nlp, alpha_bar);

  arr data;

  for(;data.d0<K;){
//    arr x = nlp.getInitializationSample();
    walk.set_alpha_bar(alpha_bar);
    walk.initialize(nlp.getUniformSample());

    bool good = false;
    for(uint t=0;t<20;t++){
      if(verbose>1){
        nlp.report(cout, 2+verbose, STRING("sample_greedy data: " <<data.d0 <<" iters: " <<t <<" good: " <<good));
      }
      walk.step();
      if(!walk.sig && walk.ev.err<=.01){ good=true; break; }
//      komo->pathConfig.setJointState(sam.x);
//      komo->view(true, STRING(k <<' ' <<t <<' ' <<sam.err <<' ' <<good));
    }

    if(walk.sig){
      walk.set_alpha_bar(1.);
      for(uint t=0;t<10;t++){
        walk.step_slack();
        if(walk.ev.err<=.01){ good=true; break; }
      }
    }

    if(good){
      data.append(walk.x);
      data.reshape(-1, nlp.getDimension());
      if(!(data.d0%10)) cout <<'.' <<std::flush;
    }

    if(verbose>1 || (good && verbose>0)){
      nlp.report(cout, 2+verbose, STRING("sample_restarts it: " <<data.d0 <<" good: " <<good));
    }
  }
  cout <<"\nsteps/sample: " <<double(walk.samples)/K <<" evals/sample: " <<double(walk.evals)/K <<" #sam: " <<data.d0 <<endl;
  return data;
}

//===========================================================================

arr sample_denoise_direct(NLP& nlp, uint K, int verbose){
  AlphaSchedule A(AlphaSchedule::_cosine, 30);
  cout <<A.alpha_bar <<endl;

  NLP_Walker walk(nlp, A.alpha_bar(-1));

  arr data;
  for(;data.d0<K;){
    walk.initialize(nlp.getUniformSample());

    for(uint t=20;t--;){
      walk.set_alpha_bar(A.alpha_bar(t));
      if(verbose>2){
        nlp.report(cout, 1+verbose, STRING("sample_denoise_direct data: " <<data.d0 <<" iters: " <<t <<" err: " <<walk.ev.err));
      }
      walk.step();
    }

    bool good=false;
    walk.set_alpha_bar(1.);
    for(uint t=0;t<10;t++){
      walk.step_slack();
      if(walk.ev.err<=.01){ good=true; break; }
    }

    if(good){
      data.append(walk.x);
      data.reshape(-1, nlp.getDimension());
      if(!(data.d0%10)) cout <<'.' <<std::flush;
    }

    if(verbose>1 || (good && verbose>0)){
      nlp.report(cout, 2+verbose, STRING("sample_denoise_direct it: " <<data.d0 <<" good: " <<good));
    }
  }
  cout <<"\nsteps/sample: " <<double(walk.samples)/K <<" evals/sample: " <<double(walk.evals)/K <<" #sam: " <<data.d0 <<endl;
  return data;
}

//===========================================================================

arr sample_greedy(NLP& nlp, uint K, int verbose, double alpha_bar){
  NLP_Walker walk(nlp, alpha_bar);

  arr data;

  for(;data.d0<K;){
    walk.set_alpha_bar(alpha_bar);
    walk.initialize(nlp.getUniformSample());

    bool good = false;
    uint t=0;
    for(;t<20;t++){
      if(verbose>2){
        nlp.report(cout, 1+verbose, STRING("sample_greedy data: " <<data.d0 <<" iters: " <<t <<" good: " <<good));
      }
      walk.step_slack();
      if(!walk.sig && walk.ev.err<=.01){ good=true; break; }
    }

    if(walk.sig){
      walk.set_alpha_bar(1.);
      for(uint t=0;t<10;t++){
        walk.step_slack();
        if(walk.ev.err<=.01){ good=true; break; }
      }
    }

    if(good){
      data.append(walk.x);
      data.reshape(-1, nlp.getDimension());
      if(!(data.d0%10)) cout <<'.' <<std::flush;
    }

    if(verbose>1 || (good && verbose>0)){
      nlp.report(cout, 2+verbose, STRING("sample_greedy data: " <<data.d0 <<" iters: " <<t <<" good: " <<good));
    }
  }
  cout <<"\nsteps/sample: " <<double(walk.samples)/K <<" evals/sample: " <<double(walk.evals)/K <<" #sam: " <<data.d0 <<endl;
  return data;
}

//===========================================================================

arr sample_denoise(NLP& nlp, uint K, int verbose){
  AlphaSchedule A(AlphaSchedule::_cosine, 50, .1);
  //  AlphaSchedule A(AlphaSchedule::_constBeta, 50, .2);
  cout <<A.alpha_bar <<endl;

  RegularizedNLP nlp_reg(nlp);
  NLP_Walker walk(nlp_reg);

  arr data;

  for(;data.d0<K;){
    arr x = nlp.getUniformSample();
    arr trace = x;
    trace.append(x);
    walk.initialize(x);
    walk.x = nlp.getUniformSample();

    for(int t=A.alpha_bar.N-1;t>0;t--){
      //get the alpha
      double bar_alpha_t = A.alpha_bar(t);
      double bar_alpha_t1 = A.alpha_bar(t-1);
      double alpha_t = bar_alpha_t/bar_alpha_t1;

      //sample an x0
      nlp_reg.setRegularization(1/sqrt(bar_alpha_t)*x, (1-bar_alpha_t)/bar_alpha_t);
      walk.step();

      //denoise
      x = (  (sqrt(bar_alpha_t1) * (1.-alpha_t)) * walk.x
             + (sqrt(alpha_t) * (1.-bar_alpha_t1)) * x )
          / (1.-bar_alpha_t);
      if(t>1){
        arr z = randn(x.N);
        x += z * sqrt( ((1.-bar_alpha_t1) * (1.-alpha_t)) / (1.-bar_alpha_t));
      }

      trace.append(x);
      trace.append(walk.x);
    }

#if 0
    trace.reshape(A.alpha_bar.N, 2*x.N);
    FILE("z.dat") <<trace.modRaw();
    gnuplot("plot 'z.dat' us 0:1 t 'x', 'z.dat' us 0:2 t 'x', 'z.dat' us 0:3 t 'x0', 'z.dat' us 0:4 t 'x0'");
    rai::wait();
#endif

    bool good = true;
//    if(walk.err>2.*walk.eps) good=false;

    if(good){
      for(uint t=0;t<10;t++){
        walk.step_slack();
        if(walk.ev.err<=.01) break;
      }
      if(walk.ev.err>.01) good=false;
    }

    if(good){
      data.append(x);
      data.reshape(-1, nlp.getDimension());
      if(!(data.d0%10)) cout <<'.' <<std::flush;
    }

    if(verbose>1 || (good && verbose>0)){
      nlp.report(cout, 2+verbose, STRING("sample_denoise it: " <<data.d0 <<" good: " <<good));
    }
  }
  cout <<"\nsteps/sample: " <<double(walk.samples)/K <<" evals/sample: " <<double(walk.evals)/K <<endl;
  return data;
}

//===========================================================================

AlphaSchedule::AlphaSchedule(AlphaSchedule::Mode mode, uint T, double beta){
  alpha_bar.resize(T+1);

  if(mode==_constBeta){
    CHECK(beta>0, "beta parameter needed");
    double alpha = 1.-beta*beta;
    for(uint t=0;t<alpha_bar.N;t++){
      alpha_bar(t) = pow(alpha, double(t));
    }
  }else if(mode==_cosine){
    double s = .01;
    double f0 = rai::sqr(::cos(s/(1.+s) * RAI_PI/2.));
    for(uint t=0;t<alpha_bar.N;t++){
      double ft = rai::sqr(::cos( ((double(t)/alpha_bar.N)+s)/(1.+s) * RAI_PI/2.));
      alpha_bar(t) = ft/f0;
    }
  }else if(mode==_linear){
    for(uint t=0;t<alpha_bar.N;t++){
      alpha_bar(t) = 1.-double(t)/(alpha_bar.N);
    }
  }else if(mode==_sqrtLinear){
    for(uint t=0;t<alpha_bar.N;t++){
      alpha_bar(t) = 1.-double(t)/(alpha_bar.N);
      alpha_bar(t) = sqrt(alpha_bar(t));
    }
  }
}

//===========================================================================

double normalCDF(double value){
   return 0.5 * erfc(-value * M_SQRT1_2);
}

double LineSampler::eval_beta(double beta){
  double p=1.;
  for(uint i=0;i<b.N;i++){
    if(fabs(s(i))>1e-6){
//      p *= rai::sigmoid( (beta-b(i))/s(i) );
      p *= normalCDF( (beta-b(i))/s(i) );
    }else{
      if(beta<b(i)){ p = 0.; break; } //single hard barrier violated
    }
  }
  p = ::pow(p, 1./num_constraints );
  return p;
}

void LineSampler::add_constraints(const arr& gbar, const arr& gd, double sig){
  uint n=b.N;
  b.append(zeros(gbar.N));
  s.append(zeros(gbar.N));
  for(uint i=0;i<gbar.N;i++){
    double gdi = gd.elem(i);
    double si = 0.;
    if(fabs(gdi)>1e-6) si = -1./gdi;
    s(n+i) = si * sig;
    b(n+i) = gbar(i) * si;
  }
  num_constraints++;
}

void LineSampler::add_constraints_eq(const arr& hbar, const arr& hd, double sig){
  uint n=b.N;
  b.append(zeros(2*hbar.N));
  s.append(zeros(2*hbar.N));
  for(uint i=0;i<hbar.N;i++){
    double hdi = hd.elem(i);
    double si = 0.;
    if(fabs(hdi)>1e-6) si = -1./hdi;
    s(n+2*i) = si * sig;
    b(n+2*i) = hbar(i) * si - rai::sign(si)*(sig+.1);
    s(n+2*i+1) = -si * sig;
    b(n+2*i+1) = hbar(i) * si + rai::sign(si)*(sig+.1);
  }
}


void LineSampler::clip_beta(const arr& gbar, const arr& gd){
  for(uint i=0;i<gbar.N;i++){
    double gdi = gd.elem(i);
    if(fabs(gdi)>1e-6){
      double beta = -gbar.elem(i) / gdi;
      if(gdi<0. && beta>beta_lo) beta_lo=beta;
      if(gdi>0. && beta<beta_up) beta_up=beta;
    }
  }
}

double LineSampler::sample_beta(){
  //-- find outer interval
  double z = 3.;
  for(uint i=0;i<b.N;i++){
    if(s(i)>0. && b(i)-z*s(i)>beta_lo) beta_lo=b(i)-z*s(i);
    if(s(i)<0. && b(i)-z*s(i)<beta_up) beta_up=b(i)-z*s(i);
  }

  if(beta_up<beta_lo) return NAN;

  //-- sample uniformly in the interval
  arr betas = rand(10);
  betas *= beta_up-beta_lo;
  betas += beta_lo;

  //-- evaluate all samples
  arr p_betas(betas.N);
  for(uint i=0;i<betas.N;i++) p_betas(i) = eval_beta(betas(i));

  //-- SUS from these samples
  arr sum_p = integral(p_betas);
  double total_p = sum_p(-1);
  if(total_p<1e-10) return NAN;
  double r = rnd.uni() * total_p;
  uint i = 0;
  for(;i<sum_p.N;i++){ if(r<sum_p(i)) break; }
  p_beta = p_betas(i);
  return betas(i);
}

double LineSampler::sample_beta_uniform(){ return beta_lo + rnd.uni()*(beta_up-beta_lo); }

void LineSampler::plot(){
  arr betas = range(-2., 2., 100);
  arr p_betas(betas.N);
  for(uint i=0;i<betas.N;i++) p_betas(i) = eval_beta(betas(i));

  FILE("z.dat") <<rai::catCol({betas, p_betas}).modRaw() <<endl;
  gnuplot("plot 'z.dat' us 1:2");
  rai::wait();
}
