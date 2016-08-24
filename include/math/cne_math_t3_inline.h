#if 0

//=========================================================================

NEINLINE neT3 neT3::operator * (const neT3 & t)
{
  neT3 ret;

  ret.rot.M[0][0] = rot.M[0][0] * t.rot.M[0][0] + rot.M[1][0] * t.rot.M[0][1] + rot.M[2][0] * t.rot.M[0][2];
  ret.rot.M[0][1] = rot.M[0][1] * t.rot.M[0][0] + rot.M[1][1] * t.rot.M[0][1] + rot.M[2][1] * t.rot.M[0][2];
  ret.rot.M[0][2] = rot.M[0][2] * t.rot.M[0][0] + rot.M[1][2] * t.rot.M[0][1] + rot.M[2][2] * t.rot.M[0][2];

  ret.rot.M[1][0] = rot.M[0][0] * t.rot.M[1][0] + rot.M[1][0] * t.rot.M[1][1] + rot.M[2][0] * t.rot.M[1][2];
  ret.rot.M[1][1] = rot.M[0][1] * t.rot.M[1][0] + rot.M[1][1] * t.rot.M[1][1] + rot.M[2][1] * t.rot.M[1][2];
  ret.rot.M[1][2] = rot.M[0][2] * t.rot.M[1][0] + rot.M[1][2] * t.rot.M[1][1] + rot.M[2][2] * t.rot.M[1][2];

  ret.rot.M[2][0] = rot.M[0][0] * t.rot.M[2][0] + rot.M[1][0] * t.rot.M[2][1] + rot.M[2][0] * t.rot.M[2][2];
  ret.rot.M[2][1] = rot.M[0][1] * t.rot.M[2][0] + rot.M[1][1] * t.rot.M[2][1] + rot.M[2][1] * t.rot.M[2][2];
  ret.rot.M[2][2] = rot.M[0][2] * t.rot.M[2][0] + rot.M[1][2] * t.rot.M[2][1] + rot.M[2][2] * t.rot.M[2][2];

  ret.pos.v[0] = rot.M[0][0] * t.pos.v[0] + rot.M[1][0] * t.pos.v[1] + rot.M[2][0] * t.pos.v[2] + pos.v[0];
  ret.pos.v[1] = rot.M[0][1] * t.pos.v[0] + rot.M[1][1] * t.pos.v[1] + rot.M[2][1] * t.pos.v[2] + pos.v[1];
  ret.pos.v[2] = rot.M[0][2] * t.pos.v[0] + rot.M[1][2] * t.pos.v[1] + rot.M[2][2] * t.pos.v[2] + pos.v[2];

/*
  ret.rot[0] = rot[0] * t.rot.M[0][0] + rot[1] * t.rot.M[0][1] + rot[2] * t.rot.M[0][2];
  ret.rot[1] = rot[0] * t.rot.M[1][0] + rot[1] * t.rot.M[1][1] + rot[2] * t.rot.M[1][2];
  ret.rot[2] = rot[0] * t.rot.M[2][0] + rot[1] * t.rot.M[2][1] + rot[2] * t.rot.M[2][2];
  
  ret.pos = rot[0] * t.pos[0] + rot[1] * t.pos[1] + rot[2] * t.pos[2] + pos;
*/
  return ret;
}

NEINLINE neV3 neT3::operator * (const neV3 & v)
{
  return rot * v + pos;
}

NEINLINE neT3 neT3::FastInverse()
{
  neT3 ret;

  ret.rot.SetTranspose(rot);

  neV3 tpos = ret.rot * pos;

  ret.pos = -tpos;

  return ret;
}
  
#endif