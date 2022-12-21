#include<iostream>

#define MAX_SIZE 3

using namespace std;

int check_full(int,int);
int check_empty(int,int);
int insert(int [],int &,int &);
int del(int [],int &,int &);
void display(int [],int,int);

int main()
{
  int ring_buffer[MAX_SIZE],choice,num;
  int head=-1;  //INITIALLY
  int tail=-1;

  do
  {
    cout<<"\n\nMENU :";
    cout<<"\n\t1. INSERT ELEMENT";
    cout<<"\n\t2. DELETE ELEMENT";
    cout<<"\n\t3. DISPLAY\n\t4. EXIT";
    cout<<"\nENTER YOUR CHOICE : ";
    cin>>choice;

    switch(choice)
    {
      case 1:
        if(!check_full(head,tail))
        {
          num=insert(ring_buffer,head,tail);
          cout<<"\nINSERTED ELEMENT : "<<num;
        }
        else
          cout<<"RING BUFFER FULL : NO SPACE TO INSERT";
        break;
      case 2:
        if(!check_empty(head,tail))
        {
          num=del(ring_buffer,head,tail);
          cout<<"\nDELETED ELEMENT : "<<num;
        }
        else
          cout<<"\nRING BUFFER EMPTY : NO ELEMENT TO DELETE";
        break;
      case 3:
        display(ring_buffer,head,tail);
        break;
      case 4:
        cout<<"THANK TOU!!!";
        break;
      default:
        cout<<"\nPLEASE ENTER A VALID OPTION";
    }
  }while(choice!=4);
}
int check_full(int h,int t)
{
  if(h==(t+1)%MAX_SIZE)
    return 1;
  else
    return 0;
}
int check_empty(int h,int t)
{
  if(h==-1 && t==-1)
    return 1;
  else
    return 0;
}
int insert(int buffer[],int &h,int &t)
{
  int element;
  cout<<"ENTER THE ELEMENT : ";
  cin>>element;
  if(h==-1 && t==-1)
  {
    t++;
    h++;
  }
  else
    t=(t+1)%MAX_SIZE;
  buffer[t]=element;
  cout<<"ELEMENT INSERTED SUCCESSFULLY";
  return element;
}
int del(int buffer[],int &h,int &t)
{
  int element;
  element=buffer[h];
  h=(h+1)%MAX_SIZE;
  cout<<"ELEMENT DELETED SUCCESSFULLY";
  if(h==(t+1)%MAX_SIZE)
    h=t=-1;
  return element;
}
void display(int buffer[],int h,int t)
{
  cout<<"RING BUFFER : ";
  if(!check_empty(h,t))
  {
    int i=h;
    while(1)
    {
      cout<<buffer[i]<<" ";
      if(i==t)
        break;
      i=(i+1)%MAX_SIZE;
    }
  }
  else
    cout<<"EMPTY";
}