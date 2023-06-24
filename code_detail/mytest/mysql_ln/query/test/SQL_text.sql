use michen_db;

create table product(
 pid int primary key auto_increment,
 pname varchar(20) not null ,
 price double, 
 category_id varchar(20)
);

insert into product values(null,'海尔洗衣机',5000,'c001');
insert into product values(null,'美的冰箱',3000,'c001');
insert into product values(null,'格力空调',5000,'c001');
insert into product values(null,'啄木鸟衬衣',300,'c002');
insert into product values(null,'恒源祥西裤',800,'c002');
insert into product values(null,'花花公子夹克',440,'c002');
insert into product values(null,'劲霸休闲裤',266,'c002');
insert into product values(null,'海澜之家卫衣',180,'c002');
insert into product values(null,'杰克琼斯运动裤',430,'c002');
insert into product values(null,'兰蔻面霜',300,'c003');
insert into product values(null,'雅诗兰黛精华水',200,'c003');
insert into product values(null,'香奈儿香水',350,'c003');
insert into product values(null,'SK-II神仙水',350,'c003');
insert into product values(null,'资生堂粉底液',180,'c003');
insert into product values(null,'老北京方便面',56,'c004');
insert into product values(null,'良品铺子海带丝',17,'c004');
insert into product values(null,'三只松鼠坚果',88,null);

select * from product where category_id != 'c003' order by pid;

select * from product where pname not like '%海%'  and price > 300 order by -price;
select * from product where pname not like '%海%'  and price > 300 order by price desc;

select * from product where pname not like '%海%'  and price * 1.2 > 500;

select category_id,count(category_id) from product group by category_id;

select category_id,max(price) max_v from product group by category_id;

select category_id,avg(price) avg_ from product group by category_id having avg_ >200 order by avg_ desc;

select max(price) - min(price) as '价格差距' from product;

